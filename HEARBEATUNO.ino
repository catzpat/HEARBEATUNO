#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"

// =========================
// OLED
// =========================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// =========================
// MAX30102
// =========================
MAX30105 particleSensor;

// =========================
// PINS
// =========================
#define BTN_PREV    3
#define BTN_NEXT    4
#define BTN_SELECT  5
#define BUZZER_PIN  A3

// =========================
// EEPROM
// =========================
#define EE_MAGIC_ADDR     0
#define EE_BPM_MIN_ADDR   1
#define EE_BPM_MAX_ADDR   2
#define EE_SPO2_MIN_ADDR  3
#define EE_SPO2_MAX_ADDR  4
#define EE_MAGIC_VALUE    91

// =========================
// USER SETTINGS
// =========================
int bpmMinAlarm  = 55;
int bpmMaxAlarm  = 120;
int spo2MinAlarm = 92;
int spo2MaxAlarm = 100;

// =========================
// CONFIG
// =========================
const long fingerThreshold = 30000;
const unsigned long buttonDebounceMs = 160;
const unsigned long displayIntervalMs = 45;
const unsigned long serialIntervalMs  = 180;
const unsigned long heartFlashMs      = 150;

const int BPM_MIN_VALID = 45;
const int BPM_MAX_VALID = 190;

// =========================
// SCREEN MODE
// =========================
enum ScreenMode {
  SCREEN_MAIN = 0,
  SCREEN_SET_BPM_MIN,
  SCREEN_SET_BPM_MAX,
  SCREEN_SET_SPO2_MIN,
  SCREEN_SET_SPO2_MAX,
  SCREEN_COUNT
};

ScreenMode currentScreen = SCREEN_MAIN;
bool editMode = false;

// =========================
// MEASURE STATE
// =========================
int bpmDisplay = 0;
int spo2Display = 0;
int spo2LastValid = 0;

bool fingerPresent = false;
bool heartFlash = false;
bool alarmActive = false;

unsigned long lastBeatMs = 0;
unsigned long lastDisplayMs = 0;
unsigned long lastSerialMs = 0;
unsigned long lastButtonMs = 0;
unsigned long lastSirenStepMs = 0;

// BPM smoothing
int bpmHistory[2] = {0, 0};
uint8_t bpmIndex = 0;
uint8_t bpmCount = 0;

// SpO2 estimate
long dcIR = 0;
long dcRed = 0;
long acIRAvg = 0;
long acRedAvg = 0;

// Alarm siren
bool sirenUp = true;
int sirenFreq = 900;

// =========================
// Custom beat detection
// =========================
long irDCBeat = 0;
long irACBeat = 0;
long irACPrev = 0;
bool rising = false;
long irMax = 0;
long irMin = 0;

bool detectBeat(long irValue) {
  // DC removal
  irDCBeat = irDCBeat + ((irValue - irDCBeat) >> 3);
  irACBeat = irValue - irDCBeat;

  bool beatDetected = false;

  // âm sang dương
  if (irACPrev < 0 && irACBeat >= 0) {
    long amplitude = irMax - irMin;

    if (amplitude > 12 && amplitude < 2500) {
      beatDetected = true;
    }

    irMax = 0;
    irMin = 0;
    rising = true;
  }

  // theo dõi đỉnh
  if (irACBeat > irACPrev && rising) {
    irMax = irACBeat;
  }

  // theo dõi đáy
  if (irACBeat < irACPrev) {
    irMin = irACBeat;
    rising = false;
  }

  irACPrev = irACBeat;
  return beatDetected;
}

void updateBPM(bool beat) {
  if (beat) {
    unsigned long now = millis();
    unsigned long delta = now - lastBeatMs;
    lastBeatMs = now;

    if (delta > 250 && delta < 1800) {
      int bpmNow = 60000 / delta;

      if (bpmNow >= BPM_MIN_VALID && bpmNow <= BPM_MAX_VALID) {
        bpmHistory[bpmIndex] = bpmNow;
        bpmIndex = (bpmIndex + 1) % 2;
        if (bpmCount < 2) bpmCount++;

        long sum = 0;
        for (uint8_t i = 0; i < bpmCount; i++) {
          sum += bpmHistory[i];
        }
        bpmDisplay = sum / bpmCount;
        heartFlash = true;
      }
    }
  }

  if (heartFlash && millis() - lastBeatMs > heartFlashMs) {
    heartFlash = false;
  }
}

// =========================
// SpO2 lookup
// =========================
const uint8_t spo2_table[184] PROGMEM = {
  95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99,
  99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
  100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97,
  97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91,
  90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 81,
  80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67,
  66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 51, 50,
  49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 31, 30, 29,
  28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5,
  3, 2, 1
};

// =========================
// EEPROM
// =========================
void loadSettings() {
  if (EEPROM.read(EE_MAGIC_ADDR) != EE_MAGIC_VALUE) {
    EEPROM.write(EE_MAGIC_ADDR, EE_MAGIC_VALUE);
    EEPROM.write(EE_BPM_MIN_ADDR, bpmMinAlarm);
    EEPROM.write(EE_BPM_MAX_ADDR, bpmMaxAlarm);
    EEPROM.write(EE_SPO2_MIN_ADDR, spo2MinAlarm);
    EEPROM.write(EE_SPO2_MAX_ADDR, spo2MaxAlarm);
    return;
  }

  bpmMinAlarm  = EEPROM.read(EE_BPM_MIN_ADDR);
  bpmMaxAlarm  = EEPROM.read(EE_BPM_MAX_ADDR);
  spo2MinAlarm = EEPROM.read(EE_SPO2_MIN_ADDR);
  spo2MaxAlarm = EEPROM.read(EE_SPO2_MAX_ADDR);

  if (bpmMinAlarm < 30 || bpmMinAlarm > 120) bpmMinAlarm = 55;
  if (bpmMaxAlarm < 80 || bpmMaxAlarm > 220) bpmMaxAlarm = 120;
  if (spo2MinAlarm < 70 || spo2MinAlarm > 99) spo2MinAlarm = 92;
  if (spo2MaxAlarm < 90 || spo2MaxAlarm > 100) spo2MaxAlarm = 100;
}

void saveSettings() {
  EEPROM.update(EE_BPM_MIN_ADDR, bpmMinAlarm);
  EEPROM.update(EE_BPM_MAX_ADDR, bpmMaxAlarm);
  EEPROM.update(EE_SPO2_MIN_ADDR, spo2MinAlarm);
  EEPROM.update(EE_SPO2_MAX_ADDR, spo2MaxAlarm);
}

// =========================
// HELPERS
// =========================
void resetMeasurements() {
  bpmDisplay = 0;
  spo2Display = 0;
  spo2LastValid = 0;

  dcIR = 0;
  dcRed = 0;
  acIRAvg = 0;
  acRedAvg = 0;

  irDCBeat = 0;
  irACBeat = 0;
  irACPrev = 0;
  rising = false;
  irMax = 0;
  irMin = 0;

  heartFlash = false;
  lastBeatMs = 0;
  bpmIndex = 0;
  bpmCount = 0;

  for (uint8_t i = 0; i < 4; i++) {
    bpmHistory[i] = 0;
  }
}

bool buttonPressed(uint8_t pin) {
  if (digitalRead(pin) == LOW && millis() - lastButtonMs > buttonDebounceMs) {
    lastButtonMs = millis();
    return true;
  }
  return false;
}

// =========================
// UI 
// =========================
void drawHeartIcon(int x, int y) {
  display.fillCircle(x + 3, y + 3, 3, SSD1306_WHITE);
  display.fillCircle(x + 7, y + 3, 3, SSD1306_WHITE);
  display.fillTriangle(x + 0, y + 5, x + 10, y + 5, x + 5, y + 11, SSD1306_WHITE);
}
void drawMainScreen() {
  char bpmText[5];
  char spo2Text[5];

  // BPM
  if (bpmDisplay >= BPM_MIN_VALID && bpmDisplay <= BPM_MAX_VALID) {
    sprintf(bpmText, "%d", bpmDisplay);   // bỏ số 0 phía trước
  } else {
    strcpy(bpmText, "");
  }

  // SpO2
  int spo2ToShow = spo2Display > 0 ? spo2Display : spo2LastValid;
  if (spo2ToShow >= 70 && spo2ToShow <= 100) {
    sprintf(spo2Text, "%d", spo2ToShow);  // bỏ số 0 phía trước
  } else {
    strcpy(spo2Text, "");
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Title
  display.setTextSize(1);
  display.setCursor(18, 2);
  display.print(F("Nhom 9 AUT1101"));


  // BPM
  display.setTextSize(2);
  display.setCursor(8, 18);
  display.print(F("BPM:"));
  display.setCursor(74, 18);
  display.print(bpmText);

  // SpO2
display.setCursor(8, 42);
display.print(F("SpO2:"));

display.setCursor(74, 42);
display.print(spo2Text);

// In dấu %
if (spo2Text[0] != '-') {
  display.print(F("%"));
}

  // Heart icon
if (fingerPresent && bpmDisplay >= BPM_MIN_VALID && bpmDisplay <= BPM_MAX_VALID && heartFlash) {
  drawHeartIcon(114, 18);
}

display.display();

  display.display();
}
void drawSettingScreen(const __FlashStringHelper* title, const __FlashStringHelper* leftLabel, const __FlashStringHelper* rightLabel, int value) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Title
  display.setTextSize(1);
  display.setCursor(26, 2);
  display.print(title);
  display.drawLine(0, 12, 127, 12, SSD1306_WHITE);

  // Labels
  display.setCursor(8, 18);
  display.print(leftLabel);

  display.setCursor(72, 18);   // lùi trái hơn để không bị xuống dòng
  display.print(rightLabel);

  // Large value
  display.setTextSize(3);
  display.setCursor(6, 34);
  display.print(value);

  // Bottom-right stacked text
  display.setTextSize(1);
  if (editMode) {
    display.setCursor(72, 40);
    display.print(F("EDIT"));
    display.setCursor(72, 48);
    display.print(F("UP:+"));
    display.setCursor(72, 56);
    display.print(F("DN:-"));

  } else {

    display.setCursor(72, 48);
    display.print(F("UP:+"));
    display.setCursor(72, 56);
    display.print(F("DN:-"));
  }

  display.display();
}
void drawCurrentScreen() {
  switch (currentScreen) {
    case SCREEN_MAIN:
      drawMainScreen();
      break;
    case SCREEN_SET_BPM_MIN:
      drawSettingScreen(F("SET BPM MIN"), F("Value"), F("BPM MIN"), bpmMinAlarm);
      break;
    case SCREEN_SET_BPM_MAX:
      drawSettingScreen(F("SET BPM MAX"), F("Value"), F("BPM MAX"), bpmMaxAlarm);
      break;
    case SCREEN_SET_SPO2_MIN:
      drawSettingScreen(F("SET SpO2 MIN"), F("Value"), F("SpO2 MIN"), spo2MinAlarm);
      break;
    case SCREEN_SET_SPO2_MAX:
      drawSettingScreen(F("SET SpO2 MAX"), F("Value"), F("SpO2 MAX"), spo2MaxAlarm);
      break;
    default:
      break;
  }
}

// =========================
// BUTTONS
// =========================
void handleButtons() {
  bool prev = buttonPressed(BTN_PREV);
  bool next = buttonPressed(BTN_NEXT);
  bool sel  = buttonPressed(BTN_SELECT);

  if (sel && currentScreen != SCREEN_MAIN) {
    editMode = !editMode;
    if (!editMode) saveSettings();
  }

  if (!editMode) {
    if (prev) {
      if (currentScreen == SCREEN_MAIN) currentScreen = SCREEN_SET_SPO2_MAX;
      else currentScreen = (ScreenMode)((int)currentScreen - 1);
    }
    if (next) {
      currentScreen = (ScreenMode)(((int)currentScreen + 1) % SCREEN_COUNT);
    }
  } else {
    switch (currentScreen) {
      case SCREEN_SET_BPM_MIN:
        if (prev && bpmMinAlarm > 30) bpmMinAlarm--;
        if (next && bpmMinAlarm < bpmMaxAlarm - 1) bpmMinAlarm++;
        break;
      case SCREEN_SET_BPM_MAX:
        if (prev && bpmMaxAlarm > bpmMinAlarm + 1) bpmMaxAlarm--;
        if (next && bpmMaxAlarm < 220) bpmMaxAlarm++;
        break;
      case SCREEN_SET_SPO2_MIN:
        if (prev && spo2MinAlarm > 70) spo2MinAlarm--;
        if (next && spo2MinAlarm < spo2MaxAlarm - 1) spo2MinAlarm++;
        break;
      case SCREEN_SET_SPO2_MAX:
        if (prev && spo2MaxAlarm > spo2MinAlarm + 1) spo2MaxAlarm--;
        if (next && spo2MaxAlarm < 100) spo2MaxAlarm++;
        break;
      default:
        break;
    }
  }
}

// =========================
// ALARM
// =========================
bool bpmAlarmTriggered() {
  return (bpmDisplay >= BPM_MIN_VALID && bpmDisplay <= BPM_MAX_VALID &&
          (bpmDisplay < bpmMinAlarm || bpmDisplay > bpmMaxAlarm));
}

bool spo2AlarmTriggered() {
  int s = spo2Display > 0 ? spo2Display : spo2LastValid;
  return (s >= 70 && s <= 100 && (s < spo2MinAlarm || s > spo2MaxAlarm));
}

void updateAlarm() {
  alarmActive = bpmAlarmTriggered() || spo2AlarmTriggered();

  if (!alarmActive) {
    noTone(BUZZER_PIN);
    return;
  }

  if (millis() - lastSirenStepMs >= 22) {
    lastSirenStepMs = millis();

    if (sirenUp) {
      sirenFreq += 40;
      if (sirenFreq >= 1450) sirenUp = false;
    } else {
      sirenFreq -= 40;
      if (sirenFreq <= 850) sirenUp = true;
    }

    tone(BUZZER_PIN, sirenFreq);
  }
}

// =========================
// SpO2
// =========================
void updateSpO2(long irValue, long redValue) {
  if (dcIR == 0) dcIR = irValue;
  else dcIR += (irValue - dcIR) / 16;

  if (dcRed == 0) dcRed = redValue;
  else dcRed += (redValue - dcRed) / 16;

  long acIR = irValue - dcIR;
  long acRed = redValue - dcRed;

  if (acIR < 0) acIR = -acIR;
  if (acRed < 0) acRed = -acRed;

  if (acIRAvg == 0) acIRAvg = acIR;
  else acIRAvg += (acIR - acIRAvg) / 8;

  if (acRedAvg == 0) acRedAvg = acRed;
  else acRedAvg += (acRed - acRedAvg) / 8;

  if (acIRAvg < 5 || acRedAvg < 5 || dcIR < 1000 || dcRed < 1000) return;

  long numerator = (acRedAvg * dcIR) / 256;
  long denominator = (dcRed * acIRAvg) / 256;
  if (denominator <= 0) return;

  int rx100 = (numerator * 100) / denominator;

  if (rx100 >= 0 && rx100 < 184) {
    int spo2Raw = pgm_read_byte_near(&spo2_table[rx100]);
    if (spo2Raw >= 70 && spo2Raw <= 100) {
      if (spo2Display == 0) spo2Display = spo2Raw;
      else spo2Display = (spo2Display * 4 + spo2Raw) / 5;
      spo2LastValid = spo2Display;
    }
  }
}

// =========================
// SENSOR
// =========================
void processSensor() {
  long irValue = particleSensor.getIR();
  long redValue = particleSensor.getRed();

  fingerPresent = irValue > fingerThreshold;

  if (!fingerPresent) {
    resetMeasurements();
    return;
  }

  bool beat = detectBeat(irValue);
  updateBPM(beat);
  updateSpO2(irValue, redValue);

if (millis() - lastSerialMs >= serialIntervalMs) {
  lastSerialMs = millis();

  Serial.println(F("===== SENSOR DATA ====="));

  Serial.print(F("Finger: "));
  Serial.println(fingerPresent ? F("YES") : F("NO"));

  Serial.print(F("IR: "));
  Serial.println(irValue);

  Serial.print(F("RED: "));
  Serial.println(redValue);

  Serial.print(F("Beat: "));
  Serial.println(beat ? F("YES") : F("NO"));

  // BPM
  Serial.print(F("BPM: "));
  if (bpmDisplay >= BPM_MIN_VALID && bpmDisplay <= BPM_MAX_VALID) {
    Serial.println(bpmDisplay);
  } else {
    Serial.println(F("--"));
  }

  // SpO2 (ưu tiên display, fallback last valid)
  int spo2ToShow = spo2Display > 0 ? spo2Display : spo2LastValid;

  Serial.print(F("SpO2: "));
  if (spo2ToShow >= 70 && spo2ToShow <= 100) {
    Serial.print(spo2ToShow);
    Serial.println(F(" %"));
  } else {
    Serial.println(F("--"));
  }

  // Alarm status
  Serial.print(F("ALARM: "));
  Serial.println(alarmActive ? F("ON") : F("OFF"));

  Serial.println(F("========================"));
}
  
}

// =========================
// SETUP
// =========================
void setup() {
  Serial.begin(115200);

  pinMode(BTN_PREV, INPUT_PULLUP);
  pinMode(BTN_NEXT, INPUT_PULLUP);
  pinMode(BTN_SELECT, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);

  loadSettings();

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    while (1);
  }

  display.clearDisplay();
  display.display();

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(18, 28);
    display.print(F("MAX30102 ERROR"));
    display.display();
    while (1);
  }

  byte ledBrightness = 0x2A;
  byte sampleAverage = 4;
  byte ledMode = 2;
  int sampleRate = 200;
  int pulseWidth = 411;
  int adcRange = 4096;

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  particleSensor.setPulseAmplitudeRed(0x24);
  particleSensor.setPulseAmplitudeIR(0x35);
  particleSensor.setPulseAmplitudeGreen(0);

  drawCurrentScreen();
}

// =========================
// LOOP
// =========================
void loop() {
  handleButtons();
  processSensor();
  updateAlarm();

  if (millis() - lastDisplayMs >= displayIntervalMs) {
    lastDisplayMs = millis();
    drawCurrentScreen();
  }
}