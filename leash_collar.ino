// Adaptive Virtual Leash - COLLAR
// Board: Seeed XIAO ESP32-C6, Arduino-ESP32 core 3.x
// Wiring: buzzer + -> D1, buzzer - -> GND, battery -> BAT+/GND pads
//
// The anchor advertises a BLE packet named "LEASH-ANCHOR" with manufacturer data:
//   [0xFF, 0xFF, threshold_ft, flags]   flags bit0 = paused
// The collar scans, reads RSSI + settings, and decides on its own.

#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// ---------- Tunables (calibrate these!) ----------
const int   BUZZER_PIN     = D1;
const char* ANCHOR_NAME    = "LEASH-ANCHOR";
const float RSSI_AT_1M     = -59.0;  // measure: RSSI with collar 1 m from anchor
const float PATH_LOSS_N    = 2.5;    // ~2.0 open air, 2.5-3.5 with walls/dog body
const float EMA_ALPHA      = 0.15;   // lower = smoother but slower
const float HYSTERESIS_FT  = 2.0;    // must come this far inside to quiet down
const float BORDER_BAND_FT = 5.0;    // chirp zone width past threshold
const float WARN_BAND_FT   = 15.0;   // distance past border where volume maxes
const uint32_t LOST_AFTER_MS = 6000; // no packets this long = signal lost
const uint32_t LOST_RAMP_MS  = 10000;
const uint8_t DEFAULT_THRESHOLD_FT = 20;
const uint8_t MAX_DUTY = 128;        // 8-bit; 128 = 50% (loudest)

// ---------- State ----------
enum Zone { SAFE, BORDER, WARNING, LOST, PAUSED };
volatile float emaRssi = NAN;
volatile uint32_t lastSeenMs = 0;
volatile uint8_t thresholdFt = DEFAULT_THRESHOLD_FT;
volatile bool paused = false;
volatile Zone zone = SAFE;
volatile float feetPast = 0;
uint32_t lostSinceMs = 0;
BLEScan* scan;

float rssiToFeet(float rssi) {
  float meters = pow(10.0, (RSSI_AT_1M - rssi) / (10.0 * PATH_LOSS_N));
  return meters * 3.281;
}

class AnchorCallback : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) {
    if (!dev.haveName() || dev.getName() != ANCHOR_NAME) return;
    float r = dev.getRSSI();
    emaRssi = isnan(emaRssi) ? r : (EMA_ALPHA * r + (1 - EMA_ALPHA) * emaRssi);
    lastSeenMs = millis();
    auto md = dev.getManufacturerData();
    if (md.length() >= 4) {
      thresholdFt = (uint8_t)md[2];
      paused = ((uint8_t)md[3] & 0x01);
    }
  }
};

// ---------- Sound ----------
void soundOff() { ledcWrite(BUZZER_PIN, 0); }
void soundOn(uint32_t freq, uint8_t duty) {
  ledcChangeFrequency(BUZZER_PIN, freq, 8);
  ledcWrite(BUZZER_PIN, duty);
}

void playBorder() {            // soft double-tick every 2 s
  uint32_t p = millis() % 2000;
  if (p < 40 || (p >= 120 && p < 160)) soundOn(3000, 30); else soundOff();
}

void playWarning(float feetPastBorder) {  // faster, louder the farther out
  float t = constrain(feetPastBorder / WARN_BAND_FT, 0.0, 1.0);
  uint8_t duty = 40 + t * (MAX_DUTY - 40);
  uint32_t period = 700 - t * 400;        // 700 ms -> 300 ms
  uint32_t p = millis() % period;
  if (p < period / 2) soundOn(3500, duty); else soundOff();
}

void playLost() {              // starts soft, ramps to max
  float t = constrain((millis() - lostSinceMs) / (float)LOST_RAMP_MS, 0.0, 1.0);
  uint8_t duty = 20 + t * (MAX_DUTY - 20);
  uint32_t p = millis() % 600;
  if (p < 250) soundOn(3200, duty); else soundOff();
}

// Sound runs in its own task so the blocking BLE scan never stalls patterns
void soundTask(void*) {
  for (;;) {
    switch (zone) {
      case BORDER:  playBorder(); break;
      case WARNING: playWarning(feetPast); break;
      case LOST:    playLost(); break;
      default:      soundOff();
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ---------- Setup / loop ----------
void setup() {
  Serial.begin(115200);
  ledcAttach(BUZZER_PIN, 3000, 8);
  soundOff();
  BLEDevice::init("");
  scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new AnchorCallback(), false);
  scan->setActiveScan(false);
  scan->setInterval(100);
  scan->setWindow(99);
  lastSeenMs = millis();  // grace period at boot
  xTaskCreate(soundTask, "sound", 2048, nullptr, 1, nullptr);
}

void loop() {
  scan->start(1, false);   // 1 s scan slices; callbacks update state
  scan->clearResults();

  uint32_t now = millis();
  bool lost = (now - lastSeenMs) > LOST_AFTER_MS;

  Zone newZone;
  if (paused) {
    newZone = PAUSED;
  } else if (lost) {
    newZone = LOST;
  } else {
    float ft = rssiToFeet(emaRssi);
    float T = thresholdFt;
    feetPast = max(0.0f, ft - T - BORDER_BAND_FT);
    if (ft < T - HYSTERESIS_FT) newZone = SAFE;
    else if (ft < T) newZone = (zone == SAFE) ? SAFE : zone;  // hysteresis hold
    else if (ft < T + BORDER_BAND_FT) newZone = BORDER;
    else newZone = WARNING;
    Serial.printf("rssi=%.1f ft=%.1f thr=%u zone=%d\n", (float)emaRssi, ft, thresholdFt, newZone);
  }
  if (newZone == LOST && zone != LOST) lostSinceMs = now;
  zone = newZone;
}
