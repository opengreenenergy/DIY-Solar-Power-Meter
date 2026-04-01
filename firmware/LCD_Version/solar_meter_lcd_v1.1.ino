/*
=============================================================
  PROJECT       : DIY Solar Power Meter
  VERSION       : v1.1 
  UPDATED ON    : 01-Apr-2026
  AUTHOR        : Open Green Energy

  LICENSE
  ------------------------------------------------------------
  Copyright (c) 2026 Open Green Energy
  Creative Commons Attribution-NonCommercial-ShareAlike 4.0
  This project is open-source for learning, research, and personal use.
  Commercial use is prohibited without written permission from Open Green Energy.

  HARDWARE USED
  -----------------------------------------------------------
  MCU           : Seeed XIAO ESP32-C3
  DISPLAY       : Waveshare 1.54 inch ST7789 SPI LCD, 240x240
  ISC SENSOR    : INA226
  IMU           : LIS2MDL Magnetometer + LSM303 Accelerometer
  TEMP SENSOR   : DS18B20
  STORAGE       : EEPROM for calibration storage

  USER OPERATION
  -----------------------------------------------------------
  Short press      : Next page
  Long press 2 sec : Start calibration

=============================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <EEPROM.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>
#include "logo_oge.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_LSM303_Accel.h>

#include <INA226.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// =============================================================
// Build options
// =============================================================
#define HEADER_WEATHER_ICON 0

// =============================================================
// Pin mapping
// =============================================================
const int PIN_BAT_SENSE   = D0;
const int PIN_MOSFET_GPIO = D1;
const int PIN_BUTTON_GPIO = D2;
const int PIN_DS18B20     = D9;

const int LCD_MOSI = D10;
const int LCD_SCK  = D8;
const int LCD_CS   = D6;
const int LCD_DC   = D7;
const int LCD_RST  = D3;

// =============================================================
// Constants
// =============================================================
const uint8_t INA_ADDR = 0x44;
const float SHUNT_OHMS = 0.50f;
const float CAL_K_WM2_PER_MA = 15.05f;

const float BAT_RTOP = 200000.0f;
const float BAT_RBOT = 100000.0f;

const unsigned long MEAS_PERIOD_MS = 800;
const unsigned long IMU_PERIOD_MS  = 150;
const unsigned long BAT_PERIOD_MS  = 1000;
const unsigned long TEMP_PERIOD_MS = 1000;
const unsigned long UI_PERIOD_MS   = 180;

const unsigned long DEBOUNCE_MS   = 18;
const unsigned long LONG_PRESS_MS = 2000;

const unsigned int ISC_SETTLE_MS     = 200;
const uint8_t ISC_SAMPLES            = 20;
const unsigned int ISC_SAMPLE_GAP_MS = 10;

const float IRR_ALPHA  = 0.25f;
const float TILT_ALPHA = 0.15f;
const float AZ_ALPHA   = 0.12f;

const unsigned long BAT_BLINK_PERIOD_MS = 900;

// =============================================================
// EEPROM
// =============================================================
const int EEPROM_BYTES = 128;
const uint32_t CAL_MAGIC = 0xA11C0A1Bu;
const uint16_t CAL_VER   = 6;

struct CalData {
  uint32_t magic;
  uint16_t ver;

  uint8_t iscValid;
  float iscOffset_mA;

  uint8_t tiltValid;
  float pitch0;
  float roll0;
  float tiltScale;

  uint8_t magValid;
  float magOffX;
  float magOffY;
  float magOffZ;

  uint8_t pad[16];
};

// =============================================================
// Display constants
// =============================================================
const int SCREEN_W = 240;
const int SCREEN_H = 240;

const int TOP_H  = 34;
const int FOOT_H = 18;

const int HDR_BATT_W = 90;
const int HDR_PAD_L  = 10;

const uint16_t C_DARKGRAY  = 0x4208;
const uint16_t C_LIGHTGRAY = 0xC618;

const int CARD_R = 10;
const int CARD_INSET = 4;

// =============================================================
// Pages
// =============================================================
enum PageId : uint8_t {
  PAGE_HOME,
  PAGE_MINMAX,
  PAGE_TEMP,
  PAGE_COMPASS
};

PageId pageSequence[4] = { PAGE_HOME, PAGE_MINMAX, PAGE_TEMP, PAGE_COMPASS };
uint8_t pageIndex = 0;

// =============================================================
// Objects
// =============================================================
Adafruit_ST7789 tft(LCD_CS, LCD_DC, LCD_RST);
INA226 ina(INA_ADDR);
Adafruit_LIS2MDL mag;
Adafruit_LSM303_Accel_Unified accel(54321);

OneWire oneWire(PIN_DS18B20);
DallasTemperature ds18b20(&oneWire);

// =============================================================
// Runtime status
// =============================================================
bool okIna = false;
bool okAcc = false;
bool okMag = false;

float batteryVoltage = 0.0f;
float batteryPercent = 0.0f;

float tempC = 0.0f;
float tempF = 32.0f;

float tiltRaw = 0.0f;
float tiltDisplay = 0.0f;

float azimuthRaw = 0.0f;
float azimuthDisplay = 0.0f;

bool iscMeasuringNow = false;

float iscRaw_mA = 0.0f;
float irrNow = 0.0f;
float irrFiltered = 0.0f;

float irrMin = NAN;
float irrMax = NAN;

unsigned long lastMeasMs = 0;
unsigned long lastImuMs  = 0;
unsigned long lastBatMs  = 0;
unsigned long lastTempMs = 0;
unsigned long lastUiMs   = 0;

// =============================================================
// Calibration
// =============================================================
float iscOffset_mA = 0.0f;
bool iscOffsetValid = false;

float tiltPitch0 = 0.0f;
float tiltRoll0  = 0.0f;
float tiltScale  = 1.0f;
bool tiltCalValid = false;

float magOffsetX = 0.0f;
float magOffsetY = 0.0f;
float magOffsetZ = 0.0f;
bool magCalValid = false;

// =============================================================
// Button
// =============================================================
bool inCalibration = false;
unsigned long buttonDownMs = 0;
bool longPressFired = false;

// =============================================================
// UI Cache
// =============================================================
struct UiCache {
  int irrInt = -99999;
  int tilt10 = -999999;
  int az10   = -999999;
  int nowI   = -99999;
  int minI   = -99999;
  int maxI   = -99999;
  int needleDeg = -999999;

  int temp10  = -999999;
  int tempF10 = -999999;

  int headerPct = -999;
  int headerIconKind = -99;
  int headerBlinkPhase = -99;
} ui;

// =============================================================
// Utility helpers
// =============================================================
float radToDeg(float rad) {
  return rad * 180.0f / (float)M_PI;
}

float clampFloat(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

float wrap360(float deg) {
  while (deg < 0) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  return deg;
}

float wrap180(float deg) {
  while (deg < -180.0f) deg += 360.0f;
  while (deg >= 180.0f) deg -= 360.0f;
  return deg;
}

// =============================================================
// EEPROM functions
// =============================================================
void loadCalibration() {
  CalData cal;
  EEPROM.get(0, cal);

  if (cal.magic != CAL_MAGIC || cal.ver != CAL_VER) {
    iscOffsetValid = false;
    iscOffset_mA = 0.0f;

    tiltCalValid = false;
    tiltPitch0 = 0.0f;
    tiltRoll0 = 0.0f;
    tiltScale = 1.0f;

    magCalValid = false;
    magOffsetX = 0.0f;
    magOffsetY = 0.0f;
    magOffsetZ = 0.0f;
    return;
  }

  iscOffsetValid = (cal.iscValid != 0);
  iscOffset_mA   = cal.iscOffset_mA;

  tiltCalValid = (cal.tiltValid != 0);
  tiltPitch0   = cal.pitch0;
  tiltRoll0    = cal.roll0;
  tiltScale    = cal.tiltScale;

  magCalValid = (cal.magValid != 0);
  magOffsetX  = cal.magOffX;
  magOffsetY  = cal.magOffY;
  magOffsetZ  = cal.magOffZ;
}

void saveCalibration() {
  CalData cal;
  memset(&cal, 0, sizeof(cal));

  cal.magic = CAL_MAGIC;
  cal.ver   = CAL_VER;

  cal.iscValid     = iscOffsetValid ? 1 : 0;
  cal.iscOffset_mA = iscOffset_mA;

  cal.tiltValid = tiltCalValid ? 1 : 0;
  cal.pitch0    = tiltPitch0;
  cal.roll0     = tiltRoll0;
  cal.tiltScale = tiltScale;

  cal.magValid = magCalValid ? 1 : 0;
  cal.magOffX  = magOffsetX;
  cal.magOffY  = magOffsetY;
  cal.magOffZ  = magOffsetZ;

  EEPROM.put(0, cal);
  EEPROM.commit();
}

// =============================================================
// Battery functions
// =============================================================
float batteryPercentFromVoltage(float v) {
  if (v >= 4.20f) return 100.0f;
  if (v <= 3.20f) return 0.0f;

  struct Pt { float v; float p; };
  static const Pt points[] = {
    {4.20f, 100.0f},
    {3.90f,  60.0f},
    {3.70f,  40.0f},
    {3.50f,  15.0f},
    {3.30f,   5.0f},
    {3.20f,   0.0f}
  };

  for (int i = 0; i < (int)(sizeof(points) / sizeof(points[0])) - 1; i++) {
    float v1 = points[i].v;
    float v2 = points[i + 1].v;

    if (v <= v1 && v >= v2) {
      float t = (v - v2) / (v1 - v2);
      float p = points[i + 1].p + t * (points[i].p - points[i + 1].p);
      return clampFloat(p, 0.0f, 100.0f);
    }
  }

  return 0.0f;
}

float readBatteryVoltage() {
  const int samples = 32;
  uint32_t sum = 0;

  for (int i = 0; i < samples; i++) {
    sum += analogRead(PIN_BAT_SENSE);
    delayMicroseconds(300);
  }

  float raw = sum / (float)samples;
  float vadc = (raw * 3.3f) / 4095.0f;
  float vbatt = vadc * (BAT_RTOP + BAT_RBOT) / BAT_RBOT;
  return vbatt;
}

// =============================================================
// DS18B20 functions
// =============================================================
float readTemperatureC() {
  ds18b20.requestTemperatures();
  float t = ds18b20.getTempCByIndex(0);

  if (t == DEVICE_DISCONNECTED_C || t < -55.0f || t > 125.0f) {
    return tempC;
  }

  return t;
}

// =============================================================
// INA226 functions
// =============================================================
void serviceWait(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    delay(1);
  }
}

float measureIsc_mA() {
  if (!okIna) return 0.0f;

  float sum = 0.0f;
  iscMeasuringNow = true;

  digitalWrite(PIN_MOSFET_GPIO, HIGH);
  serviceWait(ISC_SETTLE_MS);

  for (uint8_t i = 0; i < ISC_SAMPLES; i++) {
    float shunt_uV = ina.getShuntVoltage_uV();
    float current_mA = (shunt_uV / 1000.0f) / SHUNT_OHMS;
    sum += current_mA;
    serviceWait(ISC_SAMPLE_GAP_MS);
  }

  digitalWrite(PIN_MOSFET_GPIO, LOW);
  iscMeasuringNow = false;

  return sum / (float)ISC_SAMPLES;
}

void updateIrradiance() {
  if (!okIna) return;

  iscRaw_mA = measureIsc_mA();

  float offset = iscOffsetValid ? iscOffset_mA : 0.0f;
  float iscCorrected = iscRaw_mA - offset;
  if (iscCorrected < 0.0f) iscCorrected = 0.0f;

  irrNow = iscCorrected * CAL_K_WM2_PER_MA;
  irrFiltered = irrFiltered + IRR_ALPHA * (irrNow - irrFiltered);

  if (isnan(irrMin) || irrFiltered < irrMin) irrMin = irrFiltered;
  if (isnan(irrMax) || irrFiltered > irrMax) irrMax = irrFiltered;
}

// =============================================================
// IMU functions
// =============================================================
void updateTiltAndAzimuth() {
  if (iscMeasuringNow) return;

  float roll = 0.0f;
  float pitch = 0.0f;
  const int samples = 8;

  if (okAcc) {
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;

    for (int i = 0; i < samples; i++) {
      sensors_event_t aEvent;
      accel.getEvent(&aEvent);
      ax += aEvent.acceleration.x;
      ay += aEvent.acceleration.y;
      az += aEvent.acceleration.z;
      delay(2);
    }

    ax /= samples;
    ay /= samples;
    az /= samples;

    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 0.001f) norm = 0.001f;

    float axn = ax / norm;
    float ayn = ay / norm;
    float azn = az / norm;

    roll  = atan2f(ayn, azn);
    pitch = atan2f(-axn, sqrtf(ayn * ayn + azn * azn));

    float rollDeg  = radToDeg(roll)  - tiltRoll0;
    float pitchDeg = radToDeg(pitch) - tiltPitch0;
    float rawTilt = sqrtf(rollDeg * rollDeg + pitchDeg * pitchDeg);

    float newTilt = tiltCalValid ? (rawTilt * tiltScale) : rawTilt;
    newTilt = clampFloat(newTilt, 0.0f, 90.0f);

    if (fabsf(newTilt - tiltRaw) > 0.50f) {
      tiltRaw = newTilt;
    }

    tiltDisplay = tiltDisplay + TILT_ALPHA * (tiltRaw - tiltDisplay);
    tiltDisplay = roundf(tiltDisplay * 10.0f) / 10.0f;
  }

  if (okMag) {
    float mx = 0.0f;
    float my = 0.0f;
    float mz = 0.0f;

    for (int i = 0; i < samples; i++) {
      sensors_event_t mEvent;
      mag.getEvent(&mEvent);
      mx += mEvent.magnetic.x;
      my += mEvent.magnetic.y;
      mz += mEvent.magnetic.z;
      delay(2);
    }

    mx /= samples;
    my /= samples;
    mz /= samples;

    mx -= (magCalValid ? magOffsetX : 0.0f);
    my -= (magCalValid ? magOffsetY : 0.0f);
    mz -= (magCalValid ? magOffsetZ : 0.0f);

    float cosR = cosf(roll);
    float sinR = sinf(roll);
    float cosP = cosf(pitch);
    float sinP = sinf(pitch);

    float Xh = mx * cosP + mz * sinP;
    float Yh = mx * sinR * sinP + my * cosR - mz * sinR * cosP;

    float azimuth = radToDeg(atan2f(-Xh, Yh));
    azimuth = wrap360(azimuth + 180.0f);

    float diffRaw = wrap180(azimuth - azimuthRaw);
    if (fabsf(diffRaw) > 3.0f) {
      azimuthRaw = azimuth;
    }

    float d = wrap180(azimuthRaw - azimuthDisplay);
    azimuthDisplay = wrap360(azimuthDisplay + AZ_ALPHA * d);
    azimuthDisplay = roundf(azimuthDisplay);
  }
}

// =============================================================
// Drawing helpers
// =============================================================
void fillRectFast(int x, int y, int w, int h, uint16_t color) {
  tft.fillRect(x, y, w, h, color);
}

void drawCard(int x, int y, int w, int h, uint16_t borderColor) {
  tft.fillRoundRect(x, y, w, h, CARD_R, ST77XX_BLACK);
  tft.drawRoundRect(x, y, w, h, CARD_R, borderColor);
}

void clearCardInner(int x, int y, int w, int h) {
  const int inset = 10;
  fillRectFast(x + inset, y + inset, w - 2 * inset, h - 2 * inset, ST77XX_BLACK);
}

void drawSmallLabel(int x, int y, const char* text) {
  tft.setFont(NULL);
  tft.setTextSize(1);
  tft.setTextColor(C_LIGHTGRAY, ST77XX_BLACK);
  tft.setCursor(x, y);
  tft.print(text);
}

void drawCenteredFreeFont(const GFXfont* font, const char* text,
                          int x, int y, int w, int h,
                          uint16_t fg, uint16_t bg,
                          int yShift = 0) {
  tft.setFont(font);
  tft.setTextSize(1);
  tft.setTextColor(fg, bg);

  int16_t x1, y1;
  uint16_t tw, th;
  tft.getTextBounds((char*)text, 0, 0, &x1, &y1, &tw, &th);

  int cx = x + (w - (int)tw) / 2 - x1;
  int cy = y + (h - (int)th) / 2 - y1 + yShift;

  tft.setCursor(cx, cy);
  tft.print(text);
  tft.setFont(NULL);
}

void drawDegreeSymbol(int x, int y, uint16_t color) {
  tft.drawCircle(x, y, 3, color);
  tft.drawCircle(x, y, 2, color);
}

// =============================================================
// Boot screen
// =============================================================
void showBootScreen() {
  tft.fillScreen(ST77XX_BLACK);
  delay(10);
  tft.fillScreen(ST77XX_BLACK);
  delay(5);

  tft.drawRGBBitmap(0, 0, oge_logo, 240, 240);
  delay(20);

  int panelY = 186;
  int panelH = 54;
  tft.fillRoundRect(0, panelY, 240, panelH, 12, ST77XX_BLACK);

  tft.setFont(NULL);
  tft.setTextWrap(false);

  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setCursor(26, 198);
  tft.print("Solar Power Meter");

  tft.setTextSize(1);
  tft.setTextColor(C_LIGHTGRAY, ST77XX_BLACK);
  tft.setCursor(80, 216);
  tft.print("Version-1.2");

  int barX = 28;
  int barY = 230;
  int barW = 184;
  int barH = 4;

  tft.drawRoundRect(barX - 1, barY - 1, barW + 2, barH + 2, 2, C_DARKGRAY);
  tft.fillRoundRect(barX, barY, barW, barH, 2, 0x2104);

  int fillW = 0;
  while (fillW < barW) {
    fillW += 5;
    if (fillW > barW) fillW = barW;

    tft.fillRoundRect(barX, barY, fillW, barH, 2, ST77XX_GREEN);

    int shineX = barX + fillW - 10;
    if (shineX < barX) shineX = barX;

    int shineW = 8;
    if (shineX + shineW > barX + fillW) shineW = (barX + fillW) - shineX;

    if (shineW > 0) {
      tft.fillRect(shineX, barY, shineW, barH, ST77XX_WHITE);
      delay(15);
      tft.fillRect(shineX, barY, shineW, barH, ST77XX_GREEN);
    } else {
      delay(15);
    }
  }

  delay(250);
}

// =============================================================
// Icons
// =============================================================
int iconKindFromIrradiance(float irr) {
  if (irr < 100.0f) return 0;
  if (irr < 600.0f) return 1;
  return 2;
}

void drawIconSunSmall(int cx, int cy) {
  tft.fillCircle(cx, cy, 5, ST77XX_YELLOW);
  for (int a = 0; a < 360; a += 60) {
    float rad = a * (float)M_PI / 180.0f;
    int x1 = cx + (int)lround(cosf(rad) * 7);
    int y1 = cy + (int)lround(sinf(rad) * 7);
    int x2 = cx + (int)lround(cosf(rad) * 9);
    int y2 = cy + (int)lround(sinf(rad) * 9);
    tft.drawLine(x1, y1, x2, y2, ST77XX_YELLOW);
  }
}

void drawIconCloudSmall(int cx, int cy) {
  tft.fillCircle(cx - 5, cy + 1, 5, C_LIGHTGRAY);
  tft.fillCircle(cx + 1, cy - 2, 6, C_LIGHTGRAY);
  tft.fillCircle(cx + 8, cy + 1, 5, C_LIGHTGRAY);
  tft.fillRect(cx - 10, cy + 1, 22, 8, C_LIGHTGRAY);
}

void drawIconPartlySmall(int cx, int cy) {
  drawIconSunSmall(cx - 4, cy - 3);
  drawIconCloudSmall(cx + 3, cy + 2);
}

void drawIconSunBig(int cx, int cy) {
  tft.fillCircle(cx, cy, 12, ST77XX_YELLOW);
  for (int a = 0; a < 360; a += 30) {
    float rad = a * (float)M_PI / 180.0f;
    int x1 = cx + (int)lround(cosf(rad) * 15);
    int y1 = cy + (int)lround(sinf(rad) * 15);
    int x2 = cx + (int)lround(cosf(rad) * 20);
    int y2 = cy + (int)lround(sinf(rad) * 20);
    tft.drawLine(x1, y1, x2, y2, ST77XX_YELLOW);
  }
}

void drawIconCloudBig(int cx, int cy) {
  tft.fillCircle(cx - 12, cy + 2, 10, C_LIGHTGRAY);
  tft.fillCircle(cx, cy - 4, 12, C_LIGHTGRAY);
  tft.fillCircle(cx + 14, cy + 2, 10, C_LIGHTGRAY);
  tft.fillRoundRect(cx - 26, cy + 2, 52, 18, 9, C_LIGHTGRAY);
}

void drawIconPartlyBig(int cx, int cy) {
  drawIconSunBig(cx - 10, cy - 8);
  drawIconCloudBig(cx + 6, cy + 4);
}

// =============================================================
// Header
// =============================================================
uint16_t batteryZoneColor(int pct) {
  if (pct < 20) return ST77XX_RED;
  if (pct <= 40) return ST77XX_YELLOW;
  return ST77XX_GREEN;
}

void drawHeaderTitleFit(const char* title) {
  fillRectFast(0, 0, SCREEN_W - HDR_BATT_W, TOP_H, ST77XX_BLACK);

  int availW = (SCREEN_W - HDR_BATT_W) - HDR_PAD_L;
  if (availW < 40) availW = 40;

  tft.setFont(NULL);
  tft.setTextWrap(false);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

  int16_t x1, y1;
  uint16_t tw, th;

  tft.setTextSize(3);
  tft.getTextBounds((char*)title, 0, 0, &x1, &y1, &tw, &th);
  if ((int)tw <= availW) {
    tft.setCursor(HDR_PAD_L, 6);
    tft.print(title);
    return;
  }

  tft.setTextSize(2);
  tft.getTextBounds((char*)title, 0, 0, &x1, &y1, &tw, &th);
  if ((int)tw <= availW) {
    tft.setCursor(HDR_PAD_L, 10);
    tft.print(title);
    return;
  }

  tft.setTextSize(1);
  tft.setCursor(HDR_PAD_L, 12);
  tft.print(title);
}

void updateHeader(bool force) {
  int iconKind = iconKindFromIrradiance(irrFiltered);

  int pct = (int)lround(batteryPercent);
  pct = (int)clampFloat((float)pct, 0.0f, 100.0f);

  int blinkPhase = 0;
  if (pct < 15) {
    blinkPhase = (int)((millis() / BAT_BLINK_PERIOD_MS) % 2);
  }

  if (force || iconKind != ui.headerIconKind) {
    ui.headerIconKind = iconKind;
    fillRectFast(SCREEN_W - HDR_BATT_W + 4, 5, 18, 18, ST77XX_BLACK);

#if HEADER_WEATHER_ICON
    int cx = SCREEN_W - HDR_BATT_W + 13;
    int cy = 14;

    if (iconKind == 0) drawIconCloudSmall(cx, cy);
    else if (iconKind == 1) drawIconPartlySmall(cx, cy);
    else drawIconSunSmall(cx, cy);
#endif
  }

  bool percentChanged = (force || pct != ui.headerPct);

  if (percentChanged) {
    ui.headerPct = pct;

    fillRectFast(SCREEN_W - 70, 4, 40, TOP_H - 6, ST77XX_BLACK);

    char buf[6];
    snprintf(buf, sizeof(buf), "%d%%", pct);

    tft.setFont(NULL);
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

    int16_t x1, y1;
    uint16_t tw, th;
    tft.getTextBounds(buf, 0, 0, &x1, &y1, &tw, &th);

    int tx = (SCREEN_W - 70) + (40 - (int)tw) / 2;
    int ty = 10;

    tft.setCursor(tx, ty);
    tft.print(buf);
  }

  if (force || percentChanged || blinkPhase != ui.headerBlinkPhase) {
    ui.headerBlinkPhase = blinkPhase;

    fillRectFast(SCREEN_W - 31, 7, 28, 16, ST77XX_BLACK);

    tft.drawRect(SCREEN_W - 30, 8, 22, 12, C_LIGHTGRAY);
    tft.drawRect(SCREEN_W - 8, 12, 3, 4, C_LIGHTGRAY);

    fillRectFast(SCREEN_W - 29, 9, 20, 10, ST77XX_BLACK);

    bool blinkOff = (pct < 15) && (blinkPhase == 0);

    if (!blinkOff) {
      uint16_t fillColor = batteryZoneColor(pct);
      int fillW = 20 * pct / 100;
      fillRectFast(SCREEN_W - 29, 9, fillW, 10, fillColor);
    }
  }
}

void drawHeaderBar(const char* title) {
  fillRectFast(0, 0, SCREEN_W, TOP_H, ST77XX_BLACK);
  drawHeaderTitleFit(title);
  tft.drawFastHLine(0, TOP_H, SCREEN_W, C_DARKGRAY);

  ui.headerPct = -999;
  ui.headerIconKind = -99;
  ui.headerBlinkPhase = -99;
  updateHeader(true);
}

// =============================================================
// Footer and page dots
// =============================================================
void drawFooter(const char* hint) {
  tft.drawFastHLine(0, SCREEN_H - FOOT_H, SCREEN_W, C_DARKGRAY);
  tft.setFont(NULL);
  tft.setTextSize(1);
  tft.setTextColor(C_LIGHTGRAY, ST77XX_BLACK);
  tft.setCursor(8, SCREEN_H - FOOT_H + 5);
  tft.print(hint);
}

void drawPageDots(int activeIndex) {
  int total = 4;
  int dotR = 3;
  int gap = 10;

  int totalW = (total - 1) * gap;
  int cx0 = SCREEN_W / 2 - totalW / 2;
  int cy  = 218;

  fillRectFast(0, cy - 8, SCREEN_W, 16, ST77XX_BLACK);

  for (int i = 0; i < total; i++) {
    int cx = cx0 + i * gap;
    uint16_t color = (i == activeIndex) ? ST77XX_YELLOW : C_DARKGRAY;
    tft.fillCircle(cx, cy, dotR, color);
  }
}

// =============================================================
// Home page
// =============================================================
void drawHomeStatic() {
  tft.fillScreen(ST77XX_BLACK);
  drawHeaderBar("IRRADIANCE");

  drawCard(10, 44, 220, 96, ST77XX_CYAN);
  drawSmallLabel(20, 54, "GLOBAL HORIZONTAL IRRADIANCE");

  drawCard(10, 150, 106, 60, ST77XX_CYAN);
  drawCard(124, 150, 106, 60, ST77XX_YELLOW);
  drawSmallLabel(20, 158, "TILT");
  drawSmallLabel(134, 158, "AZ");

  drawFooter("Click: Next | Long: Cal");
  drawPageDots(0);

  ui.irrInt = -99999;
  ui.tilt10 = -999999;
  ui.az10   = -999999;
}

void drawHomeIrradiance(int irrInt) {
  clearCardInner(10, 44, 220, 96);

  int kind = iconKindFromIrradiance((float)irrInt);

  int iconX = 42;
  int iconY = 92;

  if (kind == 0 || kind == 1) iconX += 5;

  if (kind == 0) drawIconCloudBig(iconX, iconY);
  else if (kind == 1) drawIconPartlyBig(iconX, iconY);
  else drawIconSunBig(iconX, iconY);

  char buf[10];
  snprintf(buf, sizeof(buf), "%d", irrInt);

  tft.setFont(&FreeSansBold24pt7b);
  tft.setTextSize(1);

  int16_t x1, y1;
  uint16_t tw, th;
  tft.getTextBounds(buf, 0, 0, &x1, &y1, &tw, &th);

  int baseX = 85;
  int baseY = 64;
  int areaW = 150;
  int areaH = 56;

  if (kind == 2) baseX -= 5;
  else baseX += 5;
  if (irrInt >= 1000) baseX -= 5;

  int top = baseY + (areaH - (int)th) / 2;
  int baseline = top - y1;
  int startX = baseX - x1;

  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setCursor(startX, baseline);
  tft.print(buf);

  int unitX = startX + tw + 8;
  int unitY = baseline + 4;

  tft.setFont(NULL);
  tft.setTextSize(2);
  tft.setTextColor(C_LIGHTGRAY, ST77XX_BLACK);
  tft.setCursor(unitX, unitY);
  tft.print("W/m");

  tft.setTextSize(1);
  tft.setCursor(unitX + 36, unitY - 8);
  tft.print("2");
}

void drawHomeTiltAz() {
  fillRectFast(10 + CARD_INSET, 150 + 24, 106 - 2 * CARD_INSET, 60 - 30, ST77XX_BLACK);
  fillRectFast(124 + CARD_INSET, 150 + 24, 106 - 2 * CARD_INSET, 60 - 30, ST77XX_BLACK);

  char tiltBuf[16];
  char azBuf[16];

  snprintf(tiltBuf, sizeof(tiltBuf), "%.1f", tiltDisplay);
  snprintf(azBuf, sizeof(azBuf), "%d", (int)lround(azimuthDisplay));

  int tx = 10 + CARD_INSET;
  int ty = 150 + 22;
  int tw = 106 - 2 * CARD_INSET;
  int th = 60 - 22 - CARD_INSET;

  int ax = 124 + CARD_INSET;
  int ay = ty;
  int aw = tw;
  int ah = th;

  tft.setFont(&FreeSansBold18pt7b);

  int16_t x1, y1;
  uint16_t w1, h1;

  tft.getTextBounds(tiltBuf, 0, 0, &x1, &y1, &w1, &h1);
  int cx = tx + (tw - (int)w1) / 2 - x1;
  int cyTop = ty + (th - (int)h1) / 2;
  int baseY = cyTop - y1 - 2;

  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.setCursor(cx, baseY);
  tft.print(tiltBuf);

  int degX = cx + (int)w1 + 10;
  int degY = baseY - (int)h1 + 4;
  drawDegreeSymbol(degX, degY, ST77XX_CYAN);

  tft.getTextBounds(azBuf, 0, 0, &x1, &y1, &w1, &h1);
  cx = ax + (aw - (int)w1) / 2 - x1;
  cyTop = ay + (ah - (int)h1) / 2;
  baseY = cyTop - y1 - 2;

  tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  tft.setCursor(cx, baseY);
  tft.print(azBuf);

  degX = cx + (int)w1 + 8;
  degY = baseY - (int)h1 + 4;
  drawDegreeSymbol(degX, degY, ST77XX_YELLOW);

  tft.setFont(NULL);
}

void drawHomeDynamic() {
  updateHeader(false);

  int irrInt = (int)lround(irrFiltered);
  if (irrInt != ui.irrInt) {
    ui.irrInt = irrInt;
    drawHomeIrradiance(irrInt);
  }

  int tilt10 = (int)lround(tiltDisplay * 10.0f);
  int az10   = (int)lround(azimuthDisplay * 10.0f);

  if (tilt10 != ui.tilt10 || az10 != ui.az10) {
    ui.tilt10 = tilt10;
    ui.az10   = az10;
    drawHomeTiltAz();
  }
}

// =============================================================
// Min Max page
// =============================================================
void drawMinMaxStatic() {
  tft.fillScreen(ST77XX_BLACK);
  drawHeaderBar("IRRADIANCE");

  drawCard(10, 44, 220, 96, ST77XX_YELLOW);
  drawSmallLabel(20, 54, "LIVE");

  drawCard(10, 150, 106, 60, ST77XX_CYAN);
  drawCard(124, 150, 106, 60, ST77XX_GREEN);
  drawSmallLabel(20, 160, "MIN");
  drawSmallLabel(134, 160, "MAX");

  drawFooter("Click: Next");
  drawPageDots(1);

  ui.nowI = ui.minI = ui.maxI = -99999;
}

void drawMinMaxDynamic() {
  updateHeader(false);

  int nowI = (int)lround(irrFiltered);
  int minI = isnan(irrMin) ? -1 : (int)lround(irrMin);
  int maxI = isnan(irrMax) ? -1 : (int)lround(irrMax);

  if (nowI != ui.nowI) {
    ui.nowI = nowI;
    clearCardInner(10, 44, 220, 96);

    char buf[10];
    snprintf(buf, sizeof(buf), "%d", nowI);

    drawCenteredFreeFont(&FreeSansBold24pt7b, buf, 50, 58, 140, 70, ST77XX_WHITE, ST77XX_BLACK, -2);

    tft.setFont(NULL);
    tft.setTextSize(2);
    tft.setTextColor(C_LIGHTGRAY, ST77XX_BLACK);
    tft.setCursor(168, 106);
    tft.print("W/m");
    tft.setTextSize(1);
    tft.setCursor(204, 100);
    tft.print("2");
  }

  auto drawSmallBox = [&](int x, int y, int w, int h, int value, uint16_t col, bool dash) {
    fillRectFast(x + CARD_INSET, y + 20, w - 2 * CARD_INSET, h - 24, ST77XX_BLACK);

    if (dash) {
      drawCenteredFreeFont(&FreeSansBold18pt7b, "--", x + CARD_INSET, y + 18, w - 2 * CARD_INSET, h - 18, col, ST77XX_BLACK, -2);
    } else {
      char b[10];
      snprintf(b, sizeof(b), "%d", value);
      drawCenteredFreeFont(&FreeSansBold18pt7b, b, x + CARD_INSET, y + 18, w - 2 * CARD_INSET, h - 18, col, ST77XX_BLACK, -2);
    }
  };

  if (minI != ui.minI) {
    ui.minI = minI;
    drawSmallBox(10, 150, 106, 60, minI, ST77XX_CYAN, (minI < 0));
  }

  if (maxI != ui.maxI) {
    ui.maxI = maxI;
    drawSmallBox(124, 150, 106, 60, maxI, ST77XX_GREEN, (maxI < 0));
  }
}

// =============================================================
// Temperature page
// =============================================================
void drawTemperatureStatic() {
  tft.fillScreen(ST77XX_BLACK);
  drawHeaderBar("TEMPERATURE");

  drawCard(10, 44, 220, 78, ST77XX_YELLOW);
  drawCard(10, 132, 220, 78, ST77XX_CYAN);

  drawSmallLabel(20, 54, "PV MOD TEMP");
  drawSmallLabel(20, 142, "PV MOD TEMP");

  drawFooter("Click: Next");
  drawPageDots(2);

  ui.temp10  = -999999;
  ui.tempF10 = -999999;
}

void drawTemperatureValue(int x, int y, int w, int h, float value, uint16_t color, bool isC) {
  fillRectFast(x + 8, y + 20, w - 16, h - 26, ST77XX_BLACK);

  char buf[16];
  snprintf(buf, sizeof(buf), "%.1f", value);

  tft.setFont(&FreeSansBold24pt7b);
  tft.setTextSize(1);

  int16_t x1, y1;
  uint16_t tw, th;
  tft.getTextBounds(buf, 0, 0, &x1, &y1, &tw, &th);

  int contentX = x + 8;
  int contentY = y + 20;
  int contentW = w - 16;
  int contentH = h - 26;

  int baseX = contentX + (contentW - (int)tw) / 2 - x1 - 12;
  int baseY = contentY + (contentH - (int)th) / 2 - y1 + 2;

  tft.setTextColor(color, ST77XX_BLACK);
  tft.setCursor(baseX, baseY);
  tft.print(buf);

  int degX = baseX + (int)tw + 10;
  int degY = baseY - (int)th + 6;
  drawDegreeSymbol(degX, degY, color);

  tft.setFont(NULL);
  tft.setTextSize(2);
  tft.setTextColor(C_LIGHTGRAY, ST77XX_BLACK);
  tft.setCursor(degX + 10, baseY - 12);
  tft.print(isC ? "C" : "F");
}

void drawTemperatureDynamic() {
  updateHeader(false);

  float tempCDisplay = tempC;
  if (tempCDisplay > 70.0f) tempCDisplay = 70.0f;
  if (tempCDisplay < -20.0f) tempCDisplay = -20.0f;

  float tempFDisplay = tempCDisplay * 9.0f / 5.0f + 32.0f;

  int t10  = (int)lround(tempCDisplay * 10.0f);
  int tf10 = (int)lround(tempFDisplay * 10.0f);

  if (t10 != ui.temp10) {
    ui.temp10 = t10;
    drawTemperatureValue(10, 44, 220, 78, tempCDisplay, ST77XX_YELLOW, true);
  }

  if (tf10 != ui.tempF10) {
    ui.tempF10 = tf10;
    drawTemperatureValue(10, 132, 220, 78, tempFDisplay, ST77XX_CYAN, false);
  }
}

// =============================================================
// Compass page
// =============================================================
const char* azimuthToDirection(float deg) {
  deg = wrap360(deg);

  if (deg < 22.5f)   return "N";
  if (deg < 67.5f)   return "NE";
  if (deg < 112.5f)  return "E";
  if (deg < 157.5f)  return "SE";
  if (deg < 202.5f)  return "S";
  if (deg < 247.5f)  return "SW";
  if (deg < 292.5f)  return "W";
  if (deg < 337.5f)  return "NW";
  return "N";
}

void drawAzimuthValueBox(int x, int y, int w, int h, int azDeg, const char* dir) {
  fillRectFast(x, y, w, h, ST77XX_BLACK);

  char num[8];
  snprintf(num, sizeof(num), "%d", azDeg);

  const GFXfont* font = &FreeSansBold18pt7b;

  tft.setFont(NULL);
  tft.setTextSize(2);
  int16_t ax1, ay1;
  uint16_t atw, ath;
  tft.getTextBounds((char*)"Az:", 0, 0, &ax1, &ay1, &atw, &ath);

  tft.setFont(font);
  tft.setTextSize(1);
  int16_t nx1, ny1;
  uint16_t ntw, nth;
  tft.getTextBounds(num, 0, 0, &nx1, &ny1, &ntw, &nth);

  int16_t zx, zy;
  uint16_t zw, zh;
  tft.getTextBounds((char*)"0", 0, 0, &zx, &zy, &zw, &zh);

  int gap4 = (int)zw;

  int16_t dx1, dy1;
  uint16_t dtw, dth;
  tft.getTextBounds((char*)dir, 0, 0, &dx1, &dy1, &dtw, &dth);

  int smallGap = 10;
  int degW = 10;
  int totalW = (int)atw + smallGap + (int)ntw + degW + gap4 + (int)dtw;

  int startX = x + (w - totalW) / 2;
  int lineY = y + (h / 2) - 10;
  int numBaselineY = y + (h / 2) + 10;

  tft.setFont(NULL);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  tft.setCursor(startX, lineY);
  tft.print("Az:");

  int numX = startX + (int)atw + smallGap;

  tft.setFont(font);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  tft.setCursor(numX - nx1, numBaselineY);
  tft.print(num);

  int degX = (numX - nx1) + (int)ntw + 8;
  int degY = (numBaselineY - (int)nth) + 4;
  drawDegreeSymbol(degX, degY, ST77XX_YELLOW);

  int dirX = degX + degW + gap4;
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setCursor(dirX - dx1, numBaselineY);
  tft.print(dir);

  tft.setFont(NULL);
}

void drawCompassRose(int cx, int cy, int r) {
  tft.drawCircle(cx, cy, r, C_DARKGRAY);
  tft.drawCircle(cx, cy, r - 1, C_DARKGRAY);

  for (int d = 0; d < 360; d += 15) {
    float rr = d * (float)M_PI / 180.0f;
    int x1 = cx + (int)(sinf(rr) * (r - 4));
    int y1 = cy - (int)(cosf(rr) * (r - 4));
    int x2 = cx + (int)(sinf(rr) * r);
    int y2 = cy - (int)(cosf(rr) * r);

    uint16_t color = (d % 45 == 0) ? C_LIGHTGRAY : C_DARKGRAY;
    tft.drawLine(x1, y1, x2, y2, color);
  }

  int labelR = r - 16;

  tft.setFont(NULL);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

  tft.setCursor(cx - 6, cy - labelR - 8);
  tft.print("S");

  tft.setCursor(cx - 6, cy + labelR - 8);
  tft.print("N");

  tft.setCursor(cx - labelR - 6, cy - 8);
  tft.print("E");

  tft.setCursor(cx + labelR - 6, cy - 8);
  tft.print("W");
}

void drawNeedle(int cx, int cy, int r, float deg) {
  const uint16_t C_HEAD = ST77XX_RED;
  const uint16_t C_TAIL = 0x055F;

  float rad = (deg - 90.0f) * (float)M_PI / 180.0f;

  int len = r - 22;
  int w = 8;

  int xt = cx + (int)lround(cosf(rad) * len);
  int yt = cy + (int)lround(sinf(rad) * len);

  int xs = cx - (int)lround(cosf(rad) * len);
  int ys = cy - (int)lround(sinf(rad) * len);

  float pr = rad + (float)M_PI / 2.0f;
  int px = (int)lround(cosf(pr) * w);
  int py = (int)lround(sinf(pr) * w);

  tft.fillTriangle(cx + px, cy + py, cx - px, cy - py, xt, yt, C_HEAD);
  tft.drawTriangle(cx + px, cy + py, cx - px, cy - py, xt, yt, C_LIGHTGRAY);

  tft.fillTriangle(cx + px, cy + py, cx - px, cy - py, xs, ys, C_TAIL);
  tft.drawTriangle(cx + px, cy + py, cx - px, cy - py, xs, ys, C_DARKGRAY);

  tft.fillCircle(cx, cy, 10, ST77XX_BLACK);
  tft.drawCircle(cx, cy, 10, C_LIGHTGRAY);
  tft.drawCircle(cx, cy, 9, C_DARKGRAY);

  tft.fillCircle(cx, cy, 6, C_LIGHTGRAY);
  tft.drawCircle(cx, cy, 6, C_DARKGRAY);

  tft.fillCircle(cx, cy, 3, ST77XX_WHITE);
}

void drawCompassStatic() {
  tft.fillScreen(ST77XX_BLACK);
  drawHeaderBar("ORIENTATION");

  int valueY = TOP_H + 4;
  int valueH = 42;
  int compassY = valueY + valueH + 8;

  drawCard(10, valueY, 220, valueH, ST77XX_ORANGE);
  drawCard(10, compassY, 220, SCREEN_H - compassY - 8, ST77XX_ORANGE);

  drawFooter("Click: Next");
  drawPageDots(3);

  ui.needleDeg = -999999;
}

void drawCompassDynamic() {
  updateHeader(false);

  int azDeg = (int)lround(azimuthDisplay);
  azDeg = (azDeg % 360 + 360) % 360;

  if (azDeg == ui.needleDeg) return;
  ui.needleDeg = azDeg;

  int valueY = TOP_H + 4;
  int valueH = 42;
  int compassY = valueY + valueH + 8;

  clearCardInner(10, valueY, 220, valueH);

  const char* dir = azimuthToDirection(azimuthDisplay);

  int innerX = 10 + CARD_INSET;
  int innerY = valueY + CARD_INSET;
  int innerW = 220 - 2 * CARD_INSET;
  int innerH = valueH - 2 * CARD_INSET;

  drawAzimuthValueBox(innerX, innerY, innerW, innerH, azDeg, dir);

  int boxX = 10;
  int boxY = compassY;
  int boxW = 220;
  int boxH = SCREEN_H - compassY - 8;

  const int inset = 12;
  fillRectFast(boxX + inset, boxY + inset, boxW - 2 * inset, boxH - 2 * inset, ST77XX_BLACK);

  int safeX = boxX + inset;
  int safeY = boxY + inset;
  int safeW = boxW - 2 * inset;
  int safeH = boxH - 2 * inset;

  int cx = safeX + safeW / 2;
  int cy = safeY + safeH / 2;

  int r = min(safeW, safeH) / 2 + 2;
  if (r < 30) r = 30;

  drawCompassRose(cx, cy, r);

  float needleDeg = wrap360(azimuthDisplay + 180.0f);
  drawNeedle(cx, cy, r, needleDeg);
}

// =============================================================
// Page manager
// =============================================================
void resetUiCache() {
  ui = UiCache();
}

void drawPageStatic(PageId page) {
  resetUiCache();

  switch (page) {
    case PAGE_HOME:
      drawHomeStatic();
      break;

    case PAGE_MINMAX:
      drawMinMaxStatic();
      break;

    case PAGE_TEMP:
      drawTemperatureStatic();
      break;

    case PAGE_COMPASS:
      drawCompassStatic();
      break;
  }
}

void drawPageDynamic(PageId page) {
  switch (page) {
    case PAGE_HOME:
      drawHomeDynamic();
      break;

    case PAGE_MINMAX:
      drawMinMaxDynamic();
      break;

    case PAGE_TEMP:
      drawTemperatureDynamic();
      break;

    case PAGE_COMPASS:
      drawCompassDynamic();
      break;
  }
}

// =============================================================
// Calibration UI helpers
// =============================================================
void drawCalHeader(const char* title) {
  tft.fillScreen(ST77XX_BLACK);
  drawHeaderBar(title);
}

void drawCalStep(const char* text) {
  tft.setFont(NULL);
  tft.setTextSize(2);
  tft.setTextColor(C_LIGHTGRAY, ST77XX_BLACK);

  int16_t x1, y1;
  uint16_t tw, th;
  tft.getTextBounds((char*)text, 0, 0, &x1, &y1, &tw, &th);

  int x = (SCREEN_W - (int)tw) / 2 - x1;
  tft.setCursor(x, 46);
  tft.print(text);
}

void drawCalTitle(const char* text, uint16_t color) {
  tft.setFont(NULL);
  tft.setTextSize(2);
  tft.setTextColor(color, ST77XX_BLACK);

  int16_t x1, y1;
  uint16_t tw, th;
  tft.getTextBounds((char*)text, 0, 0, &x1, &y1, &tw, &th);

  int x = (SCREEN_W - (int)tw) / 2 - x1;
  tft.setCursor(x, 88);
  tft.print(text);
}

void drawCalInfo(const char* text, int y, uint16_t color = ST77XX_WHITE) {
  tft.setFont(NULL);
  tft.setTextSize(2);
  tft.setTextColor(color, ST77XX_BLACK);

  int16_t x1, y1;
  uint16_t tw, th;
  tft.getTextBounds((char*)text, 0, 0, &x1, &y1, &tw, &th);

  int x = (SCREEN_W - (int)tw) / 2 - x1;
  tft.setCursor(x, y);
  tft.print(text);
}

void drawCalProgress(int pct, uint16_t color) {
  pct = (int)clampFloat((float)pct, 0.0f, 100.0f);

  int x = 20;
  int y = 188;
  int w = 200;
  int h = 12;

  tft.drawRoundRect(x, y, w, h, 6, C_LIGHTGRAY);
  tft.fillRoundRect(x + 2, y + 2, w - 4, h - 4, 5, ST77XX_BLACK);

  int fillW = (w - 6) * pct / 100;
  tft.fillRoundRect(x + 3, y + 3, fillW, h - 6, 4, color);
}

void drawCalValue(const char* prefix, float value, const char* unit, uint16_t color) {
  char line[32];
  snprintf(line, sizeof(line), "%s%.2f %s", prefix, value, unit);

  fillRectFast(0, 150, SCREEN_W, 36, ST77XX_BLACK);
  drawCalInfo(line, 170, color);
}

void drawCalibrationSavedAnimation() {
  drawCalHeader("CALIBRATION");
  drawCalStep("DONE");
  drawCalTitle("SAVED", ST77XX_GREEN);

  drawCalInfo("Calibration Stored", 132, ST77XX_WHITE);
  drawCalInfo("All Values Saved", 160, ST77XX_GREEN);

  for (int i = 0; i < 2; i++) {
    tft.drawCircle(120, 196, 10, ST77XX_GREEN);
    tft.drawLine(116, 196, 119, 200, ST77XX_GREEN);
    tft.drawLine(119, 200, 126, 190, ST77XX_GREEN);
    delay(140);

    fillRectFast(105, 184, 30, 30, ST77XX_BLACK);
    delay(90);
  }

  tft.drawCircle(120, 196, 10, ST77XX_GREEN);
  tft.drawLine(116, 196, 119, 200, ST77XX_GREEN);
  tft.drawLine(119, 200, 126, 190, ST77XX_GREEN);
  delay(1000);
}

// =============================================================
// Calibration button wait
// =============================================================
void waitForCalibrationPress() {
  while (digitalRead(PIN_BUTTON_GPIO) == LOW) delay(10);
  delay(80);

  while (digitalRead(PIN_BUTTON_GPIO) == HIGH) delay(10);
  delay(30);

  while (digitalRead(PIN_BUTTON_GPIO) == LOW) delay(10);
  delay(120);
}

// =============================================================
// Calibration wizard
// =============================================================
void runCalibration() {
  inCalibration = true;

  drawCalHeader("CALIBRATION");
  drawCalStep("STEP 1 OF 4");
  drawCalTitle("ISC OFFSET", ST77XX_YELLOW);
  drawCalInfo("Cover the Panel", 128);
  drawCalInfo("Press Button to Save", 171);

  waitForCalibrationPress();

  if (okIna) {
    float mA = measureIsc_mA();
    if (mA < 0.0f) mA = 0.0f;
    iscOffset_mA = mA;
    iscOffsetValid = true;
  } else {
    iscOffset_mA = 0.0f;
    iscOffsetValid = false;
  }

  drawCalValue("Isc Captured:", iscOffset_mA, "mA", ST77XX_GREEN);
  delay(1000);

  drawCalHeader("CALIBRATION");
  drawCalStep("STEP 2 OF 4");
  drawCalTitle("TILT 0 DEG", ST77XX_CYAN);
  drawCalInfo("Keep Device Level", 128);
  drawCalInfo("Press Button to Save", 171);

  waitForCalibrationPress();

  if (okAcc) {
    sensors_event_t aEvent;
    accel.getEvent(&aEvent);

    float ax = aEvent.acceleration.x;
    float ay = aEvent.acceleration.y;
    float az = aEvent.acceleration.z;

    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 0.001f) norm = 0.001f;

    float axn = ax / norm;
    float ayn = ay / norm;
    float azn = az / norm;

    float roll  = atan2f(ayn, azn);
    float pitch = atan2f(-axn, sqrtf(ayn * ayn + azn * azn));

    tiltRoll0  = radToDeg(roll);
    tiltPitch0 = radToDeg(pitch);
  } else {
    tiltRoll0 = 0.0f;
    tiltPitch0 = 0.0f;
  }

  fillRectFast(0, 150, SCREEN_W, 36, ST77XX_BLACK);
  drawCalInfo("0 Position Captured", 170, ST77XX_GREEN);
  delay(1000);

  drawCalHeader("CALIBRATION");
  drawCalStep("STEP 3 OF 4");
  drawCalTitle("TILT 90 DEG", ST77XX_CYAN);
  drawCalInfo("Keep Device Vertical", 128);
  drawCalInfo("Press Button to Save", 171);

  waitForCalibrationPress();

  float raw90 = 90.0f;

  if (okAcc) {
    sensors_event_t aEvent;
    accel.getEvent(&aEvent);

    float ax = aEvent.acceleration.x;
    float ay = aEvent.acceleration.y;
    float az = aEvent.acceleration.z;

    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 0.001f) norm = 0.001f;

    float axn = ax / norm;
    float ayn = ay / norm;
    float azn = az / norm;

    float roll  = atan2f(ayn, azn);
    float pitch = atan2f(-axn, sqrtf(ayn * ayn + azn * azn));

    float rollDeg  = radToDeg(roll) - tiltRoll0;
    float pitchDeg = radToDeg(pitch) - tiltPitch0;

    raw90 = sqrtf(rollDeg * rollDeg + pitchDeg * pitchDeg);
    if (raw90 < 1.0f) raw90 = 1.0f;
  }

  tiltScale = 90.0f / raw90;
  tiltCalValid = true;

  fillRectFast(0, 150, SCREEN_W, 36, ST77XX_BLACK);
  drawCalInfo("90 Position Captured", 170, ST77XX_GREEN);
  delay(1000);

  drawCalHeader("CALIBRATION");
  drawCalStep("STEP 4 OF 4");
  drawCalTitle("COMPASS", ST77XX_ORANGE);
  drawCalInfo("Rotate Slowly", 128);

  float minX =  1e9f, minY =  1e9f, minZ =  1e9f;
  float maxX = -1e9f, maxY = -1e9f, maxZ = -1e9f;

  unsigned long t0 = millis();
  const unsigned long duration = 20000UL;

  while (millis() - t0 < duration) {
    int pct = (int)((millis() - t0) * 100UL / duration);
    drawCalProgress(pct, ST77XX_ORANGE);

    if (okMag) {
      sensors_event_t mEvent;
      mag.getEvent(&mEvent);

      float mx = mEvent.magnetic.x;
      float my = mEvent.magnetic.y;
      float mz = mEvent.magnetic.z;

      if (mx < minX) minX = mx;
      if (mx > maxX) maxX = mx;
      if (my < minY) minY = my;
      if (my > maxY) maxY = my;
      if (mz < minZ) minZ = mz;
      if (mz > maxZ) maxZ = mz;
    }

    delay(40);
  }

  if (okMag) {
    magOffsetX = 0.5f * (maxX + minX);
    magOffsetY = 0.5f * (maxY + minY);
    magOffsetZ = 0.5f * (maxZ + minZ);
    magCalValid = true;
  } else {
    magOffsetX = 0.0f;
    magOffsetY = 0.0f;
    magOffsetZ = 0.0f;
    magCalValid = false;
  }

  saveCalibration();
  drawCalibrationSavedAnimation();

  inCalibration = false;
  drawPageStatic(pageSequence[pageIndex]);
}

// =============================================================
// Button functions
// =============================================================
void nextPage() {
  pageIndex = (pageIndex + 1) % 4;
  drawPageStatic(pageSequence[pageIndex]);
}

void handleButton() {
  unsigned long now = millis();
  bool raw = digitalRead(PIN_BUTTON_GPIO);

  static bool lastRaw = true;
  static unsigned long lastChange = 0;

  if (raw != lastRaw) {
    lastRaw = raw;
    lastChange = now;
  }

  if (now - lastChange < DEBOUNCE_MS) return;
  if (inCalibration) return;

  if (raw == LOW) {
    if (buttonDownMs == 0) {
      buttonDownMs = now;
      longPressFired = false;
    } else if (!longPressFired && (now - buttonDownMs >= LONG_PRESS_MS)) {
      longPressFired = true;
      runCalibration();
    }
  } else {
    if (buttonDownMs != 0) {
      unsigned long pressMs = now - buttonDownMs;
      buttonDownMs = 0;

      if (!longPressFired && pressMs < LONG_PRESS_MS) {
        nextPage();
      }
    }
  }
}

// =============================================================
// Setup
// =============================================================
void setup() {
  pinMode(PIN_BUTTON_GPIO, INPUT_PULLUP);

  pinMode(PIN_MOSFET_GPIO, OUTPUT);
  digitalWrite(PIN_MOSFET_GPIO, LOW);

  analogReadResolution(12);

  Wire.begin();
  Wire.setClock(400000);

  SPI.begin(LCD_SCK, -1, LCD_MOSI, LCD_CS);
  tft.init(240, 240);
  tft.setRotation(1);
  tft.setSPISpeed(40000000);

  showBootScreen();

  EEPROM.begin(EEPROM_BYTES);
  loadCalibration();

  ds18b20.begin();

  okIna = ina.begin();
  okAcc = accel.begin();
  okMag = mag.begin();

  batteryVoltage = readBatteryVoltage();
  batteryPercent = batteryPercentFromVoltage(batteryVoltage);

  tempC = readTemperatureC();
  tempF = tempC * 9.0f / 5.0f + 32.0f;

  updateTiltAndAzimuth();

  if (okIna) {
    iscRaw_mA = measureIsc_mA();

    float offset = iscOffsetValid ? iscOffset_mA : 0.0f;
    float iscCorrected = iscRaw_mA - offset;
    if (iscCorrected < 0.0f) iscCorrected = 0.0f;

    irrNow = iscCorrected * CAL_K_WM2_PER_MA;
    irrFiltered = irrNow;
    irrMin = irrFiltered;
    irrMax = irrFiltered;
  } else {
    irrNow = 0.0f;
    irrFiltered = 0.0f;
    irrMin = NAN;
    irrMax = NAN;
  }

  pageIndex = 0;
  drawPageStatic(pageSequence[pageIndex]);

  unsigned long now = millis();
  lastMeasMs = now;
  lastImuMs  = now;
  lastBatMs  = now;
  lastTempMs = now;
  lastUiMs   = now;
}

// =============================================================
// Loop
// =============================================================
void loop() {
  unsigned long now = millis();

  handleButton();

  if (inCalibration) {
    delay(1);
    return;
  }

  if (now - lastBatMs >= BAT_PERIOD_MS) {
    batteryVoltage = readBatteryVoltage();
    batteryPercent = batteryPercentFromVoltage(batteryVoltage);
    lastBatMs = now;
    updateHeader(false);
  }

  if (now - lastTempMs >= TEMP_PERIOD_MS) {
    tempC = readTemperatureC();
    tempF = tempC * 9.0f / 5.0f + 32.0f;
    lastTempMs = now;
  }

  if (now - lastImuMs >= IMU_PERIOD_MS) {
    updateTiltAndAzimuth();
    lastImuMs = now;
  }

  if (now - lastMeasMs >= MEAS_PERIOD_MS) {
    updateIrradiance();
    lastMeasMs = now;
  }

  if (now - lastUiMs >= UI_PERIOD_MS) {
    drawPageDynamic(pageSequence[pageIndex]);
    lastUiMs = now;
  }

  delay(1);
}
