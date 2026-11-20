#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include <Laundry-Notify_inferencing.h>
#include "secrets.h"

#define I2C_SDA_PIN     16
#define I2C_SCL_PIN     15
#define MPU_INT_PIN     17
#define OLED_ADDR       0x3C
#define GRAVITY_MS2     9.80665f

#define BTN_PAGE_PIN    4
#define BTN_ACTION_PIN  5
#define BTN_RESET_PIN   6
#define BTN_MODE_PIN    0

#define WOM_THRESHOLD             12
#define WOM_DURATION              1

const int   VOTE_WINDOW         = 5;
const float MIN_CONFIDENCE      = 0.55f;

const unsigned long WAKE_VERIFY_MS  = 12000;
const int   IDLE_SLEEP_WINDOWS      = 3;
const unsigned long OLED_TIMEOUT_MS = 20000;
const unsigned long USER_AWAKE_MS   = 25000;
const unsigned long LONG_PRESS_MS   = 1800;

const int REFILL_PRESETS_SEC[] = {240, 360, 120, 60};
const int NUM_PRESETS          = 4;

RTC_DATA_ATTR int  rtcRefillPresetIdx = 0;
RTC_DATA_ATTR bool rtcWasFinished     = false;
RTC_DATA_ATTR unsigned long rtcLastWashSec = 0;

Adafruit_SH1106G display(128, 64, &Wire, -1);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

uint8_t mpuAddr = 0;
float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
String voteHistory[VOTE_WINDOW];
int voteIndex = 0;

String currentStableState = "idle";
String lastEventState     = "";
String lastRawLabel       = "idle";
float  lastConfidence     = 1.0f;
float  lastVibRms         = 0.0f;
String toastMsg           = "";
unsigned long toastUntilMs = 0;

bool isMachineRunning     = false;
bool hasSpunInCycle       = false;
bool finishNotified       = false;
bool isOledInitialized    = false;
bool isOledOn             = false;
bool stayAwakeMode        = false;
bool wokeByButton         = false;
bool wokeByMotion         = false;
int  idleVerifyCount      = 0;
uint8_t currentPage       = 0;

unsigned long bootTimeMs               = 0;
unsigned long idleStartTime            = 0;
unsigned long washStartTimeMs          = 0;
unsigned long washElapsedSec           = 0;
unsigned long oledLastWakeTime         = 0;
unsigned long lastInteractionMs        = 0;
unsigned long lastMqttReconnectAttempt = 0;

enum BtnEvent { BTN_NONE = 0, BTN_SHORT = 1, BTN_LONG = 2 };

struct SmartButton {
  uint8_t pin;
  bool hasLongPress;
  bool lastRaw;
  bool stableState;
  unsigned long lastChangeMs;
  unsigned long pressStartMs;
  bool longFired;
};

SmartButton btnPage   = {BTN_PAGE_PIN,   false, HIGH, HIGH, 0, 0, false};
SmartButton btnAction = {BTN_ACTION_PIN, false, HIGH, HIGH, 0, 0, false};
SmartButton btnReset  = {BTN_RESET_PIN,  true,  HIGH, HIGH, 0, 0, false};
SmartButton btnMode   = {BTN_MODE_PIN,   true,  HIGH, HIGH, 0, 0, false};

BtnEvent readSmartButton(SmartButton &b) {
  bool raw = digitalRead(b.pin);
  unsigned long now = millis();

  if (raw != b.lastRaw) {
    b.lastChangeMs = now;
    b.lastRaw = raw;
  }

  if (now - b.lastChangeMs > 35) {
    if (raw != b.stableState) {
      b.stableState = raw;
      if (b.stableState == LOW) {
        b.pressStartMs = now;
        b.longFired = false;
        if (!b.hasLongPress) return BTN_SHORT;
      } else {
        if (b.hasLongPress && !b.longFired) return BTN_SHORT;
      }
    }
  }

  if (b.hasLongPress && b.stableState == LOW && !b.longFired) {
    if (now - b.pressStartMs >= LONG_PRESS_MS) {
      b.longFired = true;
      return BTN_LONG;
    }
  }
  return BTN_NONE;
}

unsigned long getRefillTimeoutMs() {
  return (unsigned long)REFILL_PRESETS_SEC[rtcRefillPresetIdx] * 1000UL;
}

unsigned long getPostSpinTimeoutMs() {
  return (REFILL_PRESETS_SEC[rtcRefillPresetIdx] <= 60) ? 20000UL : 60000UL;
}

void showToast(const String &msg, unsigned long durationMs = 2000) {
  toastMsg = msg;
  toastUntilMs = millis() + durationMs;
}

void writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readReg(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)1, (uint8_t)true);
  return Wire.available() ? Wire.read() : 0;
}

bool wakeSensor() {
  for (uint8_t addr : {0x68, 0x69}) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      mpuAddr = addr;
      writeReg(mpuAddr, 0x6B, 0x00);
      delay(35);
      writeReg(mpuAddr, 0x38, 0x00);
      readReg(mpuAddr, 0x3A);
      writeReg(mpuAddr, 0x6C, 0x07);
      writeReg(mpuAddr, 0x1A, 0x04);
      writeReg(mpuAddr, 0x1C, 0x08);
      writeReg(mpuAddr, 0x69, 0x00);
      return true;
    }
  }
  return false;
}

void enableWakeOnMotion() {
  if (mpuAddr == 0) return;

  writeReg(mpuAddr, 0x6B, 0x00);
  delay(20);
  writeReg(mpuAddr, 0x6C, 0x07);
  writeReg(mpuAddr, 0x38, 0x00);
  writeReg(mpuAddr, 0x1A, 0x00);

  writeReg(mpuAddr, 0x1C, 0x00);
  delay(60);
  writeReg(mpuAddr, 0x1C, 0x07);
  writeReg(mpuAddr, 0x1F, WOM_THRESHOLD);
  writeReg(mpuAddr, 0x20, WOM_DURATION);
  writeReg(mpuAddr, 0x69, 0x15);

  writeReg(mpuAddr, 0x37, 0x20);
  readReg(mpuAddr, 0x3A);
  writeReg(mpuAddr, 0x38, 0x40);

  writeReg(mpuAddr, 0x6C, 0x47);
  writeReg(mpuAddr, 0x6B, 0x28);
  delay(25);
  readReg(mpuAddr, 0x3A);
}

void readAccelMS2(float &ax, float &ay, float &az) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(mpuAddr, (uint8_t)6, (uint8_t)true);

  if (Wire.available() >= 6) {
    uint8_t xH = Wire.read(), xL = Wire.read();
    uint8_t yH = Wire.read(), yL = Wire.read();
    uint8_t zH = Wire.read(), zL = Wire.read();

    int16_t rawX = (int16_t)((xH << 8) | xL);
    int16_t rawY = (int16_t)((yH << 8) | yL);
    int16_t rawZ = (int16_t)((zH << 8) | zL);

    ax = (rawX / 8192.0f) * GRAVITY_MS2;
    ay = (rawY / 8192.0f) * GRAVITY_MS2;
    az = (rawZ / 8192.0f) * GRAVITY_MS2;
  }
}

void wakeOLED() {
  if (!isOledInitialized) {
    display.begin(OLED_ADDR, true);
    display.setRotation(2);
    isOledInitialized = true;
  }
  if (!isOledOn) {
    display.oled_command(SH110X_DISPLAYON);
    isOledOn = true;
  }
  oledLastWakeTime = millis();
}

void sleepOLED() {
  if (isOledInitialized && isOledOn) {
    display.clearDisplay();
    display.display();
    display.oled_command(SH110X_DISPLAYOFF);
    isOledOn = false;
  }
}

String getCurrentDisplayState() {
  if (finishNotified || rtcWasFinished) return "finished";
  if (isMachineRunning && currentStableState == "idle") return "paused";
  if (!isMachineRunning && currentStableState == "idle") return "standby";
  return currentStableState;
}

String mapToWebAppState(String displayState) {
  if (displayState == "washing")  return "WASHING";
  if (displayState == "spinning") return "SPINNING";
  if (displayState == "paused")   return "SOAK_WAIT";
  if (displayState == "finished") return "DONE";
  return "IDLE";
}

void updateOLED(String displayState, unsigned long pauseElapsedMs, unsigned long activeTimeoutMs) {
  if (!stayAwakeMode && isOledOn && (millis() - oledLastWakeTime > OLED_TIMEOUT_MS)) {
    sleepOLED();
    return;
  }
  if (!isOledOn) return;

  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.printf("P%d ", currentPage + 1);
  if (stayAwakeMode) {
    display.print("[HOLD]");
  } else {
    display.print("LAUNDRY");
  }
  display.setCursor(82, 0);
  display.print(mqttClient.connected() ? "[TB:ON]" : "[TB:--]");
  display.drawLine(0, 10, 128, 10, SH110X_WHITE);

  bool hasToast = (toastMsg.length() > 0 && millis() < toastUntilMs);

  if (currentPage == 0) {
    display.setTextSize(2);
    display.setCursor(0, 16);
    String webState = mapToWebAppState(displayState);
    if (webState == "SOAK_WAIT") display.println("SOAK WAIT");
    else if (webState == "IDLE") display.println("READY");
    else display.println(webState);

    display.setTextSize(1);
    display.setCursor(0, 40);
    display.printf("AI:%s (%.0f%%)", lastRawLabel.c_str(), lastConfidence * 100.0f);

    display.setCursor(0, 54);
    if (hasToast) {
      display.print(toastMsg);
    } else if (displayState == "finished") {
      display.print("BTN4: Confirm Unload");
    } else if (displayState == "paused") {
      unsigned long remainSec = (activeTimeoutMs > pauseElapsedMs) ? ((activeTimeoutMs - pauseElapsedMs) / 1000) : 0;
      display.printf("%s: %lus", hasSpunInCycle ? "End in" : "Refill", remainSec);
    } else if (isMachineRunning) {
      display.printf("Time: %02lu:%02lu %s", washElapsedSec / 60, washElapsedSec % 60, hasSpunInCycle ? "[SPIN]" : "");
    } else {
      display.printf("WoM:%dm | BTN3:Next", REFILL_PRESETS_SEC[rtcRefillPresetIdx] / 60);
    }
  } else if (currentPage == 1) {
    display.setTextSize(1);
    display.setCursor(0, 14);
    display.printf("Vib RMS : %.3f m/s2", lastVibRms);
    display.setCursor(0, 26);
    display.printf("AI Raw  : %s %.0f%%", lastRawLabel.c_str(), lastConfidence * 100.0f);
    display.setCursor(0, 38);
    display.printf("Spun    : %s  T:%lum", hasSpunInCycle ? "YES" : "NO", washElapsedSec / 60);
    display.setCursor(0, 52);
    if (hasToast) {
      display.print(toastMsg);
    } else if (WiFi.status() == WL_CONNECTED) {
      display.printf("WiFi RSSI: %d dBm", WiFi.RSSI());
    } else {
      display.print("WiFi: Offline/Standby");
    }
  } else {
    display.setTextSize(1);
    display.setCursor(0, 14);
    int presetMin = REFILL_PRESETS_SEC[rtcRefillPresetIdx] / 60;
    display.printf("B4 Refill: %d min%s", presetMin, (presetMin == 1) ? " TEST" : "");
    display.setCursor(0, 26);
    display.printf("B2 Mode  : %s", stayAwakeMode ? "HOLD ON" : "WoM SLEEP");
    display.setCursor(0, 38);
    display.printf("WoM Thr  : %d (IO%d)", WOM_THRESHOLD, MPU_INT_PIN);
    display.setCursor(0, 52);
    display.print(hasToast ? toastMsg : "B5 Sync / B2H=Sleep");
  }

  display.display();
}

void ensureWiFiAndMQTTConnected() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("[WiFi] Connecting");
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int retry = 0;
    while (WiFi.status() != WL_CONNECTED && retry < 20) {
      delay(300);
      Serial.print(".");
      retry++;
    }
    Serial.println(WiFi.status() == WL_CONNECTED ? " OK!" : " Failed");
  }

  if (WiFi.status() == WL_CONNECTED && !mqttClient.connected()) {
    unsigned long now = millis();
    if (now - lastMqttReconnectAttempt > 3000 || lastMqttReconnectAttempt == 0) {
      lastMqttReconnectAttempt = now;
      mqttClient.setServer(TB_SERVER, TB_PORT);
      mqttClient.setBufferSize(768);
      String clientId = "ESP32S3-WoM-" + String((uint32_t)ESP.getEfuseMac(), HEX);
      if (mqttClient.connect(clientId.c_str(), TB_ACCESS_TOKEN, "")) {
        Serial.println("[ThingsBoard] Connected");
      }
    }
  }
}

void sendTelemetryRealTime(String displayState, String rawLabel, float confidence, float vibRms, bool running, bool finished, unsigned long remainSec, unsigned long elapsedSec, String eventMsg = "") {
  ensureWiFiAndMQTTConnected();
  if (!mqttClient.connected()) return;

  String webAppState = mapToWebAppState(displayState);
  float roundedRms   = round(vibRms * 1000.0f) / 1000.0f;

  JsonDocument doc;
  doc["machine_state"]      = webAppState;
  doc["vibration_rms"]      = roundedRms;
  doc["vib_rms"]            = roundedRms;
  doc["rms"]                = roundedRms;
  doc["soak_remaining_sec"] = remainSec;
  doc["remain_sec"]         = remainSec;
  doc["wash_elapsed_sec"]   = elapsedSec;
  doc["wifi_rssi"]          = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;

  doc["state"]              = displayState;
  doc["raw_label"]          = rawLabel;
  doc["confidence"]         = round(confidence * 1000.0f) / 10.0f;
  doc["is_running"]         = running;
  doc["finished"]           = finished;
  doc["has_spun"]           = hasSpunInCycle;
  doc["stay_awake"]         = stayAwakeMode;
  doc["refill_preset_sec"]  = REFILL_PRESETS_SEC[rtcRefillPresetIdx];

  if (eventMsg.length() > 0) {
    doc["event_msg"] = eventMsg;
  }

  String payload;
  serializeJson(doc, payload);
  mqttClient.publish("v1/devices/me/telemetry", payload.c_str());
  mqttClient.loop();
}

void enterWoMDeepSleep(String reason, bool sendStandbyFirst = false) {
  if (sendStandbyFirst) {
    rtcWasFinished = false;
    rtcLastWashSec = 0;
    sendTelemetryRealTime("standby", "idle", 1.0f, 0.005f, false, false, 0, 0, "เครื่องซักผ้าพร้อมทำงาน (IDLE)");
    delay(250);
  }

  if (mqttClient.connected()) {
    mqttClient.disconnect();
  }
  sleepOLED();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  enableWakeOnMotion();

  int intLevel = digitalRead(MPU_INT_PIN);
  if (intLevel == HIGH) {
    delay(60);
    readReg(mpuAddr, 0x3A);
    intLevel = digitalRead(MPU_INT_PIN);
  }

  Serial.printf("[WoM SLEEP] %s | INT(GPIO%d)=%d | Thr=%d\n",
                reason.c_str(), MPU_INT_PIN, intLevel, WOM_THRESHOLD);
  Serial.flush();

  rtc_gpio_pullup_en(GPIO_NUM_4);
  rtc_gpio_pulldown_dis(GPIO_NUM_4);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_4, 0);

  rtc_gpio_pulldown_en((gpio_num_t)MPU_INT_PIN);
  rtc_gpio_pullup_dis((gpio_num_t)MPU_INT_PIN);
  esp_sleep_enable_ext1_wakeup((1ULL << MPU_INT_PIN), ESP_EXT1_WAKEUP_ANY_HIGH);

  esp_deep_sleep_start();
}

void resetCycleToIdle(bool syncCloud, const String &toastText) {
  isMachineRunning   = false;
  hasSpunInCycle     = false;
  finishNotified     = false;
  rtcWasFinished     = false;
  idleStartTime      = 0;
  washStartTimeMs    = 0;
  washElapsedSec     = 0;
  rtcLastWashSec     = 0;
  currentStableState = "idle";
  lastRawLabel       = "idle";
  for (int i = 0; i < VOTE_WINDOW; i++) voteHistory[i] = "idle";

  showToast(toastText, 2500);
  wakeOLED();
  if (syncCloud) {
    sendTelemetryRealTime("standby", "idle", 1.0f, lastVibRms, false, false, 0, 0, toastText);
  }
}

bool handleButtons() {
  BtnEvent evPage   = readSmartButton(btnPage);
  BtnEvent evAction = readSmartButton(btnAction);
  BtnEvent evReset  = readSmartButton(btnReset);
  BtnEvent evMode   = readSmartButton(btnMode);

  if (evPage == BTN_NONE && evAction == BTN_NONE && evReset == BTN_NONE && evMode == BTN_NONE) {
    return false;
  }

  lastInteractionMs = millis();

  if (!isOledOn) {
    wakeOLED();
    showToast("Screen Awakened", 1200);
    return true;
  }
  wakeOLED();

  if (evPage == BTN_SHORT) {
    currentPage = (currentPage + 1) % 3;
    Serial.printf("[BTN3] Switch to Page %d\n", currentPage + 1);
  }

  if (evAction == BTN_SHORT) {
    if (finishNotified || rtcWasFinished) {
      resetCycleToIdle(true, "Unload OK -> IDLE");
      currentPage = 0;
    } else {
      rtcRefillPresetIdx = (rtcRefillPresetIdx + 1) % NUM_PRESETS;
      int m = REFILL_PRESETS_SEC[rtcRefillPresetIdx] / 60;
      showToast("Refill Wait: " + String(m) + "m", 2000);
      currentPage = 2;
      Serial.printf("[BTN4] Refill preset = %d sec\n", REFILL_PRESETS_SEC[rtcRefillPresetIdx]);
    }
  }

  if (evReset == BTN_SHORT) {
    String ds = getCurrentDisplayState();
    unsigned long activeTimeout = hasSpunInCycle ? getPostSpinTimeoutMs() : getRefillTimeoutMs();
    unsigned long pauseElapsed  = (idleStartTime > 0) ? (millis() - idleStartTime) : 0;
    unsigned long remainSec     = (ds == "paused" && activeTimeout > pauseElapsed) ? ((activeTimeout - pauseElapsed) / 1000) : 0;
    showToast("Syncing Cloud...", 1500);
    updateOLED(ds, pauseElapsed, activeTimeout);
    sendTelemetryRealTime(ds, lastRawLabel, lastConfidence, lastVibRms, isMachineRunning, (finishNotified || rtcWasFinished), remainSec, washElapsedSec, "Manual Sync (BTN5)");
    showToast(mqttClient.connected() ? "Cloud Synced!" : "Sync Failed!", 2000);
  } else if (evReset == BTN_LONG) {
    resetCycleToIdle(true, "Force Reset -> IDLE");
    currentPage = 0;
  }

  if (evMode == BTN_SHORT) {
    stayAwakeMode = !stayAwakeMode;
    showToast(stayAwakeMode ? "HOLD Mode: ON" : "WoM Sleep: ON", 2000);
    Serial.printf("[BTN2] Stay Awake = %s\n", stayAwakeMode ? "ON" : "OFF");
  } else if (evMode == BTN_LONG) {
    showToast("WoM Sleep...", 1000);
    updateOLED(getCurrentDisplayState(), 0, getRefillTimeoutMs());
    unsigned long waitRel = millis();
    while (digitalRead(BTN_MODE_PIN) == LOW && (millis() - waitRel < 2000)) {
      delay(10);
    }
    delay(450);
    enterWoMDeepSleep("Manual WoM sleep by BTN2", false);
  }

  return true;
}

String getMajorityState(String newPrediction) {
  voteHistory[voteIndex] = newPrediction;
  voteIndex = (voteIndex + 1) % VOTE_WINDOW;

  int countIdle = 0, countWash = 0, countSpin = 0;
  for (int i = 0; i < VOTE_WINDOW; i++) {
    if (voteHistory[i] == "idle") countIdle++;
    else if (voteHistory[i] == "washing") countWash++;
    else if (voteHistory[i] == "spinning") countSpin++;
  }

  if (countSpin >= countWash && countSpin >= countIdle) return "spinning";
  if (countWash >= countIdle && countWash >= countSpin) return "washing";
  return "idle";
}

void setup() {
  setCpuFrequencyMhz(80);
  Serial.begin(115200);
  bootTimeMs = millis();
  lastInteractionMs = bootTimeMs;

  rtc_gpio_deinit(GPIO_NUM_4);
  rtc_gpio_deinit((gpio_num_t)MPU_INT_PIN);

  pinMode(BTN_PAGE_PIN,   INPUT_PULLUP);
  pinMode(BTN_ACTION_PIN, INPUT_PULLUP);
  pinMode(BTN_RESET_PIN,  INPUT_PULLUP);
  pinMode(BTN_MODE_PIN,   INPUT_PULLUP);
  pinMode(MPU_INT_PIN,    INPUT_PULLDOWN);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);

  esp_sleep_wakeup_cause_t wakeupReason = esp_sleep_get_wakeup_cause();
  wokeByButton = (wakeupReason == ESP_SLEEP_WAKEUP_EXT0);
  wokeByMotion = (wakeupReason == ESP_SLEEP_WAKEUP_EXT1);

  if (!wakeSensor()) {
    wakeOLED();
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 20);
    display.println("SENSOR ERROR!");
    display.display();
    delay(3000);
    esp_sleep_enable_timer_wakeup(10ULL * 1000000ULL);
    esp_deep_sleep_start();
  }

  for (int i = 0; i < VOTE_WINDOW; i++) voteHistory[i] = "idle";
  if (rtcWasFinished) {
    finishNotified = true;
    washElapsedSec = rtcLastWashSec;
  }

  if (wokeByButton) {
    Serial.println("[WAKE] EXT0: Button BTN3 pressed -> Waking screen!");
    wakeOLED();
    showToast("Woke by BTN3", 1500);
    updateOLED(getCurrentDisplayState(), 0, getRefillTimeoutMs());

    unsigned long waitReleaseStart = millis();
    while (digitalRead(BTN_PAGE_PIN) == LOW && (millis() - waitReleaseStart < 1500)) {
      delay(10);
    }
    btnPage.lastRaw = btnPage.stableState = HIGH;
  } else if (wokeByMotion) {
    Serial.println("[WAKE] EXT1: Motion detected by MPU6050 (WoM)! Verifying with AI...");
  } else {
    Serial.println("[BOOT] Cold start (Wake-on-Motion AI + Buttons)");
    rtcWasFinished = false;
    rtcLastWashSec = 0;
    finishNotified = false;

    wakeOLED();
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 14);
    display.println("LAUNDRY AI [WoM]");
    display.setCursor(0, 30);
    display.println("Connecting Cloud...");
    display.setCursor(0, 48);
    display.println("Press BTN3 anytime");
    display.display();

    sendTelemetryRealTime("standby", "idle", 1.0f, 0.005f, false, false, 0, 0, "เปิดเครื่องพร้อมทำงาน (WoM IDLE)");
    lastInteractionMs = millis();
    wokeByButton = true;
    showToast("WoM Ready! BTN3", 2500);
  }
}

void loop() {
  if (mqttClient.connected()) {
    mqttClient.loop();
  }

  float sumSqDyn = 0.0f;
  bool interruptedByButton = false;

  for (size_t ix = 0; ix < EI_CLASSIFIER_RAW_SAMPLE_COUNT; ix++) {
    uint64_t nextTick = micros() + ((uint64_t)EI_CLASSIFIER_INTERVAL_MS * 1000ULL);

    float ax, ay, az;
    readAccelMS2(ax, ay, az);

    features[ix * 3 + 0] = ax;
    features[ix * 3 + 1] = ay;
    features[ix * 3 + 2] = az;

    float mag = sqrtf(ax * ax + ay * ay + az * az);
    float dyn = mag - GRAVITY_MS2;
    sumSqDyn += dyn * dyn;

    while (micros() < nextTick) {
      if (handleButtons()) {
        interruptedByButton = true;
        break;
      }
    }
    if (interruptedByButton) break;
  }

  if (interruptedByButton) {
    unsigned long activeTimeout = hasSpunInCycle ? getPostSpinTimeoutMs() : getRefillTimeoutMs();
    unsigned long pauseElapsed  = (idleStartTime > 0) ? (millis() - idleStartTime) : 0;
    updateOLED(getCurrentDisplayState(), pauseElapsed, activeTimeout);
    return;
  }

  lastVibRms = sqrtf(sumSqDyn / EI_CLASSIFIER_RAW_SAMPLE_COUNT);

  signal_t signal;
  if (numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal) != 0) return;

  ei_impulse_result_t result = { 0 };
  if (run_classifier(&signal, &result, false) != EI_IMPULSE_OK) return;

  String bestLabel = "uncertain";
  float bestScore = 0.0f;
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    if (result.classification[ix].value > bestScore) {
      bestScore = result.classification[ix].value;
      bestLabel = String(result.classification[ix].label);
    }
  }

  String normLabel = bestLabel;
  normLabel.toLowerCase();
  if (normLabel.indexOf("spin") >= 0)      bestLabel = "spinning";
  else if (normLabel.indexOf("wash") >= 0) bestLabel = "washing";
  else if (normLabel.indexOf("idle") >= 0) bestLabel = "idle";

  lastRawLabel   = bestLabel;
  lastConfidence = bestScore;

  if (wokeByMotion && !isMachineRunning && !stayAwakeMode) {
    if (bestLabel == "idle" || bestScore < MIN_CONFIDENCE) {
      idleVerifyCount++;
      if (idleVerifyCount >= IDLE_SLEEP_WINDOWS) {
        enterWoMDeepSleep(rtcWasFinished ? "Laundry unloaded -> Reset IDLE" : "Short bump / False WoM wake", rtcWasFinished);
      }
    } else {
      idleVerifyCount = 0;
      wakeOLED();
    }
  }

  String effectivePrediction = (bestScore >= MIN_CONFIDENCE) ? bestLabel : currentStableState;
  currentStableState = getMajorityState(effectivePrediction);

  if (!isMachineRunning && !stayAwakeMode) {
    if (wokeByButton) {
      if (millis() - lastInteractionMs >= USER_AWAKE_MS) {
        enterWoMDeepSleep("User screen timeout", false);
      }
    } else if (millis() - bootTimeMs >= WAKE_VERIFY_MS) {
      enterWoMDeepSleep(rtcWasFinished ? "Laundry unloaded -> Reset IDLE" : "WoM verify timeout", rtcWasFinished);
    }
  }

  unsigned long pauseElapsed    = 0;
  unsigned long activeTimeoutMs = hasSpunInCycle ? getPostSpinTimeoutMs() : getRefillTimeoutMs();
  String eventMsg = "";

  if (currentStableState == "washing" || currentStableState == "spinning") {
    idleStartTime   = 0;
    idleVerifyCount = 0;

    if (currentStableState == "spinning") {
      hasSpunInCycle = true;
    }

    if (!isMachineRunning) {
      isMachineRunning = true;
      finishNotified   = false;
      rtcWasFinished   = false;
      washStartTimeMs  = millis();
      washElapsedSec   = 0;
      hasSpunInCycle   = (currentStableState == "spinning");
      wokeByButton     = false;
      wokeByMotion     = false;
      wakeOLED();
      eventMsg = "เครื่องซักผ้าเริ่มทำงานแล้ว (" + currentStableState + ")";
    }

    washElapsedSec = (millis() - washStartTimeMs) / 1000;
  } else if (currentStableState == "idle" && isMachineRunning) {
    if (idleStartTime == 0) {
      idleStartTime = millis();
    }
    pauseElapsed   = millis() - idleStartTime;
    washElapsedSec = (millis() - washStartTimeMs) / 1000;

    if (pauseElapsed >= activeTimeoutMs && !finishNotified) {
      isMachineRunning = false;
      finishNotified   = true;
      rtcWasFinished   = true;
      rtcLastWashSec   = washElapsedSec;
      idleStartTime    = 0;
      wakeOLED();
      eventMsg = "ซักผ้าเสร็จเรียบร้อยแล้ว! สามารถนำผ้าออกได้ครับ";
    }
  }

  String displayState = getCurrentDisplayState();

  if (displayState != lastEventState) {
    lastEventState = displayState;
    wakeOLED();
  }

  unsigned long remainSec = 0;
  if (displayState == "paused" && activeTimeoutMs > pauseElapsed) {
    remainSec = (activeTimeoutMs - pauseElapsed) / 1000;
  }

  Serial.printf("Raw:%-8s(%.2f) | Web:%-9s | RMS:%.3f | Elap:%lus | Rem:%lus\n",
                bestLabel.c_str(), bestScore, mapToWebAppState(displayState).c_str(),
                lastVibRms, washElapsedSec, remainSec);

  updateOLED(displayState, pauseElapsed, activeTimeoutMs);

  if (isMachineRunning || stayAwakeMode || eventMsg.length() > 0) {
    sendTelemetryRealTime(displayState, bestLabel, bestScore, lastVibRms, isMachineRunning, (finishNotified || rtcWasFinished), remainSec, washElapsedSec, eventMsg);
  }

  if (finishNotified && eventMsg.length() > 0 && !stayAwakeMode) {
    for (int i = 0; i < 6; i++) {
      unsigned long waitStart = millis();
      while (millis() - waitStart < 2000) {
        if (handleButtons()) {
          updateOLED(getCurrentDisplayState(), 0, activeTimeoutMs);
          if (stayAwakeMode || !finishNotified) return;
        }
        delay(20);
      }
      sendTelemetryRealTime("finished", "idle", 1.0f, 0.005f, false, true, 0, washElapsedSec, "");
    }
    enterWoMDeepSleep("Wash cycle finished", false);
  }
}