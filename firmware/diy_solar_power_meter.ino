/*
=============================================================
  PROJECT       : DIY Solar Power Meter
  VERSION       : v1.1  (Professional LCD UI)
  UPDATED ON    : 08-Mar-2026
  AUTHOR        : Open Green Energy 
 ============================================================
  LICENSE       : 
  Copyright (c) 2026 Open Green Energy 
  Creative Commons Attribution-NonCommercial-ShareAlike 4.0
  This project is open-source for learning, research, and personal use.
  Commercial use is prohibited without written permission
  from Open Green Energy.  

  HARDWARE USED
  -----------------------------------------------------------
  MCU           : Seeed XIAO ESP32-C3
  DISPLAY       : Waveshare 1.54 inch LCD, 240x240, ST7789 SPI
  ISC SENSOR    : INA226
  IMU           : LIS2MDL Magnetometer + LSM303 Accelerometer
  STORAGE       : EEPROM (stores calibration values)

  WHAT THIS DEVICE DOES
  -----------------------------------------------------------
  1. Measures short circuit current from a small reference solar
     panel using INA226.
  2. Converts that current into irradiance in W/m2.
  3. Measures tilt using the accelerometer.
  4. Measures azimuth / compass direction using the magnetometer.
  5. Shows all values on a 240x240 LCD with multiple pages.
  6. Saves calibration values in EEPROM.

  WIRING GUIDE
  -----------------------------------------------------------
  1) LCD (Waveshare 1.54 inch ST7789, SKU 18079)
     LCD  ->  XIAO ESP32-C3
     VCC  ->  3V3
     GND  ->  GND
     DIN  ->  D10 (MOSI)
     CLK  ->  D8  (SCK)
     CS   ->  D6
     DC   ->  D7
     RST  ->  D3
     BL   ->  D9

  2) Button
     One side -> D2
     Other    -> GND
     INPUT_PULLUP is used, so:
     Released  = HIGH
     Pressed   = LOW

  3) INA226
     VCC  -> 3V3
     GND  -> GND
     SDA  -> SDA
     SCL  -> SCL
     ADDR -> 0x44

  4) Battery Sense Divider
     VBAT -> 200k -> D0 -> 100k -> GND

  USER OPERATION
  -----------------------------------------------------------
  Short press      : Go to next page
  Long press 2 sec : Start calibration wizard

  DEMO MODE
  -----------------------------------------------------------
  DEMO_MODE = 1  -> Fixed demo values are shown on screen
  DEMO_MODE = 0  -> Real sensor readings are used

  HEADER WEATHER ICON LOGIC
  -----------------------------------------------------------
  Irradiance < 100 W/m2      -> Cloud icon
  Irradiance 100 to 599 W/m2 -> Partly cloudy icon
  Irradiance >= 600 W/m2     -> Sun icon

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

// =============================================================
// User selectable settings
// Change these values to enable debug or demo behaviour.
// =============================================================
#define ENABLE_SERIAL_DEBUG 1
#define DEMO_MODE 1
#define HEADER_WEATHER_ICON 0   // 1 = show header weather icon, 0 = hide it

// =============================================================
// Pin mapping
// These pins match the wiring used in this project.
// =============================================================
static const int PIN_BAT_SENSE   = D0;
static const int PIN_MOSFET_GPIO = D1;
static const int PIN_BUTTON_GPIO = D2;

static const int LCD_MOSI = D10;
static const int LCD_SCK  = D8;
static const int LCD_CS   = D6;
static const int LCD_DC   = D7;
static const int LCD_RST  = D3;
static const int LCD_BL   = D9;

// =============================================================
// Electrical constants
// These values are used in measurement calculations.
// =============================================================
static const uint8_t INA_ADDR = 0x44;
static const float   SHUNT_OHMS = 0.50f;
static const float   CAL_K_WM2_PER_MA = 15.05f;   // Irradiance conversion factor

// Battery divider resistors for ADC voltage measurement
static const float BAT_RTOP = 200000.0f;
static const float BAT_RBOT = 100000.0f;

// =============================================================
// Timing values
// These decide how often different parts of the system update.
// =============================================================
static const unsigned long MEAS_PERIOD_MS = 800;   // Irradiance update period
static const unsigned long IMU_PERIOD_MS  = 120;   // Tilt / azimuth update period
static const unsigned long BAT_PERIOD_MS  = 1000;  // Battery update period
static const unsigned long UI_PERIOD_MS   = 180;   // Screen refresh period

// Button timing
static const unsigned long DEBOUNCE_MS   = 18;
static const unsigned long LONG_PRESS_MS = 2000;

// INA226 short circuit current measurement timing
static const unsigned int ISC_SETTLE_MS     = 200;
static const uint8_t      ISC_SAMPLES       = 20;
static const unsigned int ISC_SAMPLE_GAP_MS = 10;

// Display smoothing factors
// Higher value = faster response, lower value = smoother display
static const float IRR_ALPHA  = 0.25f;
static const float TILT_ALPHA = 0.45f;
static const float AZ_ALPHA   = 0.35f;

// Low battery blinking period
static const unsigned long BAT_BLINK_PERIOD_MS = 900;

// =============================================================
// EEPROM storage information
// Calibration values are stored here so they remain after power off.
// =============================================================
static const int EEPROM_BYTES = 128;
static const uint32_t CAL_MAGIC = 0xA11C0A1Bu;
static const uint16_t CAL_VER   = 6;

// This structure holds all calibration values.
struct CalData {
  uint32_t magic;
  uint16_t ver;
  uint8_t  iscValid;
  float    iscOffset_mA;

  uint8_t  tiltValid;
  float    pitch0;
  float    roll0;
  float    tiltScale;

  uint8_t  magValid;
  float    magOffX;
  float    magOffY;
  float    magOffZ;

  uint8_t pad[16];
};

// =============================================================
// Display and sensor objects
// =============================================================
Adafruit_ST7789 tft(LCD_CS, LCD_DC, LCD_RST);
INA226 ina(INA_ADDR);
Adafruit_LIS2MDL mag;
Adafruit_LSM303_Accel_Unified accel(54321);

// =============================================================
// Display layout constants
// =============================================================
static const int W = 240;
static const int H = 240;

static const int TOP_H  = 34;
static const int FOOT_H = 18;

static const int HDR_BATT_W = 90;
static const int HDR_PAD_L  = 10;

// Common colors
static const uint16_t C_DARKGRAY  = 0x4208;
static const uint16_t C_LIGHTGRAY = 0xC618;

// Card style settings
static const int CARD_R = 10;
static const int CARD_INSET = 4;

// =============================================================
// Page sequence
// The UI has 4 pages.
// =============================================================
enum PageId : uint8_t { PAGE_HOME, PAGE_MINMAX, PAGE_TILT_AZ, PAGE_COMPASS };
static const PageId PAGE_SEQ[4] = { PAGE_HOME, PAGE_MINMAX, PAGE_TILT_AZ, PAGE_COMPASS };
static uint8_t seqIndex = 0;

// =============================================================
// Runtime state
// These variables hold live values used by the system.
// =============================================================
static bool okIna = false;
static bool okAcc = false;
static bool okMag = false;

static float vbat = 0.0f;
static float battPct = 0.0f;

static float tiltDeg_raw  = 0.0f;
static float tiltDeg_disp = 0.0f;

static float azimDeg_raw  = 0.0f;
static float azimDeg_disp = 0.0f;

static float iscmA_raw = 0.0f;
static float irr_now   = 0.0f;
static float irr_filt  = 0.0f;

static float irrMin = NAN;
static float irrMax = NAN;

static unsigned long lastMeasMs = 0;
static unsigned long lastImuMs  = 0;
static unsigned long lastBatMs  = 0;
static unsigned long lastUiMs   = 0;

// Calibration values currently in use
static float iscOffset_mA = 0.0f;
static bool  iscOffsetValid = false;

static float tiltPitch0 = 0.0f;
static float tiltRoll0  = 0.0f;
static float tiltScale  = 1.0f;
static bool  tiltCalValid = false;

static float magOffsetX = 0.0f;
static float magOffsetY = 0.0f;
static float magOffsetZ = 0.0f;
static bool  magCalValid = false;

// Button state tracking
static bool inCalibration = false;
static unsigned long btnDownMs = 0;
static bool longPressFired = false;

// =============================================================
// Partial redraw cache
// This helps reduce flicker by redrawing only changed values.
// =============================================================
struct UiCache {
  int irrInt = -99999;
  int tilt10 = -999999;
  int az10   = -999999;
  int nowI   = -99999;
  int minI   = -99999;
  int maxI   = -99999;
  int isc10  = -999999;
  int needleDeg = -999999;

  int headerPct = -999;
  int headerIconKind = -99;
  int headerBlinkPhase = -99;
} ui;

// =============================================================
// Math helper functions
// =============================================================
static float rad2deg(float r) { return r * 180.0f / (float)M_PI; }

// Keep angle inside 0 to 359.999...
static float wrap360(float deg) {
  while (deg < 0) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  return deg;
}

// Keep angle inside -180 to +179.999...
static float wrap180(float deg) {
  while (deg < -180.0f) deg += 360.0f;
  while (deg >= 180.0f) deg -= 360.0f;
  return deg;
}

// Limit a value between minimum and maximum
static float clampf(float x, float a, float b) {
  if (x < a) return a;
  if (x > b) return b;
  return x;
}

// =============================================================
// Battery percentage estimation
// This is an approximate Li-ion style voltage-to-percent curve.
// =============================================================
static float batteryPercentFromVoltageCurve(float v) {
  if (v >= 4.20f) return 100.0f;
  if (v <= 3.20f) return 0.0f;

  struct Pt { float v; float p; };
  static const Pt pts[] = {
    {4.20f, 100.0f},
    {3.90f,  60.0f},
    {3.70f,  40.0f},
    {3.50f,  15.0f},
    {3.30f,   5.0f},
    {3.20f,   0.0f}
  };

  for (int i = 0; i < (int)(sizeof(pts) / sizeof(pts[0])) - 1; i++) {
    float v1 = pts[i].v;
    float v2 = pts[i + 1].v;
    if (v <= v1 && v >= v2) {
      float t = (v - v2) / (v1 - v2);
      float p = pts[i + 1].p + t * (pts[i].p - pts[i + 1].p);
      return clampf(p, 0.0f, 100.0f);
    }
  }
  return 0.0f;
}

// Read battery voltage through ADC and resistor divider
static float readBatteryVoltage() {
  unsigned int raw = analogRead(PIN_BAT_SENSE);
  float vadc = (raw * 3.3f) / 4095.0f;
  float vb = vadc * (BAT_RTOP + BAT_RBOT) / BAT_RBOT;
  return vb;
}

// =============================================================
// EEPROM helper functions
// =============================================================

// Load saved calibration values from EEPROM
static void loadCalFromEEPROM() {
  CalData cd;
  EEPROM.get(0, cd);

  // If EEPROM does not contain valid data, use defaults
  if (cd.magic != CAL_MAGIC || cd.ver != CAL_VER) {
    iscOffsetValid = false;  iscOffset_mA = 0.0f;
    tiltCalValid   = false;  tiltPitch0 = 0.0f; tiltRoll0 = 0.0f; tiltScale = 1.0f;
    magCalValid    = false;  magOffsetX = magOffsetY = magOffsetZ = 0.0f;
    return;
  }

  iscOffsetValid = (cd.iscValid != 0);
  iscOffset_mA   = cd.iscOffset_mA;

  tiltCalValid = (cd.tiltValid != 0);
  tiltPitch0   = cd.pitch0;
  tiltRoll0    = cd.roll0;
  tiltScale    = cd.tiltScale;

  magCalValid = (cd.magValid != 0);
  magOffsetX  = cd.magOffX;
  magOffsetY  = cd.magOffY;
  magOffsetZ  = cd.magOffZ;
}

// Save current calibration values into EEPROM
static void saveCalToEEPROM() {
  CalData cd;
  cd.magic = CAL_MAGIC;
  cd.ver   = CAL_VER;

  cd.iscValid     = iscOffsetValid ? 1 : 0;
  cd.iscOffset_mA = iscOffset_mA;

  cd.tiltValid = tiltCalValid ? 1 : 0;
  cd.pitch0    = tiltPitch0;
  cd.roll0     = tiltRoll0;
  cd.tiltScale = tiltScale;

  cd.magValid = magCalValid ? 1 : 0;
  cd.magOffX  = magOffsetX;
  cd.magOffY  = magOffsetY;
  cd.magOffZ  = magOffsetZ;

  EEPROM.put(0, cd);
  EEPROM.commit();
}

// =============================================================
// INA226 measurement functions
// =============================================================

// Small helper used during waiting periods
static void serviceDuringWait(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) delay(1);
}

// Measure short circuit current in mA safely
// The MOSFET is turned ON only during the measurement period.
static float measureIsc_mA_safe() {
  if (!okIna) return 0.0f;

  float sum = 0.0f;

  digitalWrite(PIN_MOSFET_GPIO, HIGH);
  serviceDuringWait(ISC_SETTLE_MS);

  for (uint8_t i = 0; i < ISC_SAMPLES; i++) {
    float shunt_uV = ina.getShuntVoltage_uV();
    float mA = (shunt_uV / 1000.0f) / SHUNT_OHMS;
    sum += mA;
    serviceDuringWait(ISC_SAMPLE_GAP_MS);
  }

  digitalWrite(PIN_MOSFET_GPIO, LOW);
  return sum / (float)ISC_SAMPLES;
}

// Update irradiance using measured current and stored offset
static void updateIrradiance() {
  if (!okIna) return;

  iscmA_raw = measureIsc_mA_safe();

  float offset = iscOffsetValid ? iscOffset_mA : 0.0f;
  float iscCorr = iscmA_raw - offset;
  if (iscCorr < 0.0f) iscCorr = 0.0f;

  irr_now  = iscCorr * CAL_K_WM2_PER_MA;
  irr_filt = irr_filt + IRR_ALPHA * (irr_now - irr_filt);

  if (isnan(irrMin) || irr_filt < irrMin) irrMin = irr_filt;
  if (isnan(irrMax) || irr_filt > irrMax) irrMax = irr_filt;
}

// =============================================================
// IMU update functions
// =============================================================

// Read tilt from accelerometer and azimuth from magnetometer
static void updateTiltAndAzimuth() {
  float roll = 0.0f, pitch = 0.0f;

  // ---------- Tilt from accelerometer ----------
  if (okAcc) {
    sensors_event_t aEvent;
    accel.getEvent(&aEvent);

    float ax = aEvent.acceleration.x;
    float ay = aEvent.acceleration.y;
    float az = aEvent.acceleration.z;

    // Normalize acceleration vector
    float an = sqrtf(ax * ax + ay * ay + az * az);
    if (an < 0.001f) an = 0.001f;

    float axn = ax / an;
    float ayn = ay / an;
    float azn = az / an;

    // Calculate roll and pitch angles
    roll  = atan2f(ayn, azn);
    pitch = atan2f(-axn, sqrtf(ayn * ayn + azn * azn));

    // Apply tilt calibration
    float rollDeg  = rad2deg(roll)  - tiltRoll0;
    float pitchDeg = rad2deg(pitch) - tiltPitch0;
    float rawMag = sqrtf(rollDeg * rollDeg + pitchDeg * pitchDeg);

    tiltDeg_raw  = tiltCalValid ? (rawMag * tiltScale) : rawMag;
    tiltDeg_raw = clampf(tiltDeg_raw, 0.0f, 90.0f);

    // Smooth the displayed value
    tiltDeg_disp = tiltDeg_disp + TILT_ALPHA * (tiltDeg_raw - tiltDeg_disp);
  }

  // ---------- Azimuth from magnetometer ----------
  if (okMag) {
    sensors_event_t mEvent;
    mag.getEvent(&mEvent);

    float mx = mEvent.magnetic.x - (magCalValid ? magOffsetX : 0.0f);
    float my = mEvent.magnetic.y - (magCalValid ? magOffsetY : 0.0f);
    float mz = mEvent.magnetic.z - (magCalValid ? magOffsetZ : 0.0f);

    // Tilt compensation
    float cosR = cosf(roll),  sinR = sinf(roll);
    float cosP = cosf(pitch), sinP = sinf(pitch);

    float Xh = mx * cosP + mz * sinP;
    float Yh = mx * sinR * sinP + my * cosR - mz * sinR * cosP;

    float azimuth = rad2deg(atan2f(-Xh, Yh));
    azimuth = wrap360(azimuth + 180.0f);

    azimDeg_raw = azimuth;

    // Smooth azimuth using shortest angle difference
    float d = wrap180(azimDeg_raw - azimDeg_disp);
    azimDeg_disp = wrap360(azimDeg_disp + AZ_ALPHA * d);
  }
}

// =============================================================
// Basic drawing helper functions
// =============================================================
static void fillRect(int x, int y, int w, int h, uint16_t c) {
  tft.fillRect(x, y, w, h, c);
}

static void drawCard(int x, int y, int w, int h, uint16_t border) {
  tft.fillRoundRect(x, y, w, h, CARD_R, ST77XX_BLACK);
  tft.drawRoundRect(x, y, w, h, CARD_R, border);
}

// Clear only the inside area of a card, so border stays visible
static void clearCardInner(int x, int y, int w, int h) {
  const int inset = 10;
  fillRect(x + inset, y + inset, w - 2 * inset, h - 2 * inset, ST77XX_BLACK);
}

// Draw a small label above a value box
static void drawSmallLabel(int x, int y, const char* s) {
  tft.setFont(NULL);
  tft.setTextSize(1);
  tft.setTextColor(C_LIGHTGRAY, ST77XX_BLACK);
  tft.setCursor(x, y);
  tft.print(s);
}

// Draw centered text using a FreeFont
static void drawCenteredFreeFont(const GFXfont* font, const char* s,
                                 int x, int y, int w, int h,
                                 uint16_t fg, uint16_t bg,
                                 int yShiftPx = 0) {
  tft.setFont(font);
  tft.setTextSize(1);
  tft.setTextColor(fg, bg);

  int16_t x1, y1;
  uint16_t tw, th;
  tft.getTextBounds((char*)s, 0, 0, &x1, &y1, &tw, &th);

  int cx = x + (w - (int)tw) / 2 - x1;
  int cy = y + (h - (int)th) / 2 - y1 + yShiftPx;

  tft.setCursor(cx, cy);
  tft.print(s);
  tft.setFont(NULL);
}

// Simple degree symbol for large text
static void drawDegreeSymbol(int x, int y, uint16_t col) {
  tft.drawCircle(x, y, 3, col);
  tft.drawCircle(x, y, 2, col);
}

// =============================================================
// Backlight helper functions
// =============================================================

// Set LCD backlight brightness
static void setBacklightSmooth(uint8_t val) {
  analogWrite(LCD_BL, val);
}

// Fade backlight from one value to another
static void fadeBacklightIn(uint8_t fromVal, uint8_t toVal, int stepDelayMs) {
  if (toVal < fromVal) return;
  for (int v = fromVal; v <= toVal; v += 8) {
    setBacklightSmooth(v);
    delay(stepDelayMs);
  }
  setBacklightSmooth(toVal);
}

// =============================================================
// Boot screen
// Shows logo, title and animated loading bar.
// =============================================================
static void showBootScreen() {
  setBacklightSmooth(0);

  tft.fillScreen(ST77XX_BLACK);
  delay(10);
  tft.fillScreen(ST77XX_BLACK);
  delay(5);

  tft.drawRGBBitmap(0, 0, oge_logo, 240, 240);
  fadeBacklightIn(0, 255, 5);
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
  tft.print("Version-1.1");

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

    // Small shine effect moving with the loading bar
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
// Weather / sky icon helpers
// Used on the home page and optionally in header.
// =============================================================
static int iconKindFromIrr(float irr) {
  if (irr < 100.0f) return 0;
  if (irr < 600.0f) return 1;
  return 2;
}

static void drawIconSunSmall(int cx, int cy) {
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

static void drawIconCloudSmall(int cx, int cy) {
  tft.fillCircle(cx - 5, cy + 1, 5, C_LIGHTGRAY);
  tft.fillCircle(cx + 1, cy - 2, 6, C_LIGHTGRAY);
  tft.fillCircle(cx + 8, cy + 1, 5, C_LIGHTGRAY);
  tft.fillRect(cx - 10, cy + 1, 22, 8, C_LIGHTGRAY);
}

static void drawIconPartlySmall(int cx, int cy) {
  drawIconSunSmall(cx - 4, cy - 3);
  drawIconCloudSmall(cx + 3, cy + 2);
}

static void drawIconSunBig(int cx, int cy) {
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

static void drawIconCloudBig(int cx, int cy) {
  tft.fillCircle(cx - 12, cy + 2, 10, C_LIGHTGRAY);
  tft.fillCircle(cx,      cy - 4, 12, C_LIGHTGRAY);
  tft.fillCircle(cx + 14, cy + 2, 10, C_LIGHTGRAY);
  tft.fillRoundRect(cx - 26, cy + 2, 52, 18, 9, C_LIGHTGRAY);
}

static void drawIconPartlyBig(int cx, int cy) {
  drawIconSunBig(cx - 10, cy - 8);
  drawIconCloudBig(cx + 6, cy + 4);
}

// =============================================================
// Header drawing
// =============================================================

// Battery color changes based on charge percentage
static uint16_t battZoneColor(int pct) {
  if (pct < 20) return ST77XX_RED;
  if (pct <= 40) return ST77XX_YELLOW;
  return ST77XX_GREEN;
}

// Draw title and automatically reduce font size if needed
static void drawHeaderTitleFit(const char* title) {
  fillRect(0, 0, W - HDR_BATT_W, TOP_H, ST77XX_BLACK);

  int availW = (W - HDR_BATT_W) - HDR_PAD_L;
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

// =============================================================
// Header live update
// Only the changing parts are redrawn to reduce flicker.
// =============================================================
static void headerUpdateStable(bool force) {
  int iconKind = iconKindFromIrr(irr_filt);

  int pct = (int)lround(battPct);
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;

  int blinkPhase = 0;
  if (pct < 15) {
    blinkPhase = (int)((millis() / BAT_BLINK_PERIOD_MS) % 2);
  }

  // Update weather icon if needed
  if (force || iconKind != ui.headerIconKind) {
    ui.headerIconKind = iconKind;

    fillRect(W - HDR_BATT_W + 4, 5, 18, 18, ST77XX_BLACK);

#if HEADER_WEATHER_ICON
    int cx = W - HDR_BATT_W + 13;
    int cy = 14;

    if (iconKind == 0)      drawIconCloudSmall(cx, cy);
    else if (iconKind == 1) drawIconPartlySmall(cx, cy);
    else                    drawIconSunSmall(cx, cy);
#endif
  }

  bool percentChanged = (force || pct != ui.headerPct);

  // Update numeric battery percentage
  if (percentChanged) {
    ui.headerPct = pct;

    fillRect(W - 70, 4, 40, TOP_H - 6, ST77XX_BLACK);

    char buf[6];
    snprintf(buf, sizeof(buf), "%d%%", pct);

    tft.setFont(NULL);
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

    int16_t x1, y1;
    uint16_t tw, th;
    tft.getTextBounds(buf, 0, 0, &x1, &y1, &tw, &th);

    int tx = (W - 70) + (40 - (int)tw) / 2;
    int ty = 10;

    tft.setCursor(tx, ty);
    tft.print(buf);
  }

  // Update battery icon fill level
  if (force || percentChanged || blinkPhase != ui.headerBlinkPhase) {
    ui.headerBlinkPhase = blinkPhase;

    fillRect(W - 31, 7, 28, 16, ST77XX_BLACK);

    tft.drawRect(W - 30, 8, 22, 12, C_LIGHTGRAY);
    tft.drawRect(W - 8, 12, 3, 4, C_LIGHTGRAY);

    fillRect(W - 29, 9, 20, 10, ST77XX_BLACK);

    bool blinkOff = (pct < 15) && (blinkPhase == 0);

    if (!blinkOff) {
      uint16_t fillCol = battZoneColor(pct);
      int fillW = 20 * pct / 100;
      fillRect(W - 29, 9, fillW, 10, fillCol);
    }
  }
}

// Draw the full header bar once
static void headerBar(const char* title) {
  fillRect(0, 0, W, TOP_H, ST77XX_BLACK);
  drawHeaderTitleFit(title);
  tft.drawFastHLine(0, TOP_H, W, C_DARKGRAY);

  ui.headerPct = -999;
  ui.headerIconKind = -99;
  ui.headerBlinkPhase = -99;
  headerUpdateStable(true);
}

// =============================================================
// Footer and page dots
// =============================================================
static void footerHint(const char* hint) {
  tft.drawFastHLine(0, H - FOOT_H, W, C_DARKGRAY);
  tft.setFont(NULL);
  tft.setTextSize(1);
  tft.setTextColor(C_LIGHTGRAY, ST77XX_BLACK);
  tft.setCursor(8, H - FOOT_H + 5);
  tft.print(hint);
}

// Small dots at bottom showing current page
static void drawPageDots(int activeIndex) {
  int total = 4;
  int dotR = 3;
  int gap = 10;

  int totalW = (total - 1) * gap;
  int cx0 = W / 2 - totalW / 2;
  int cy  = 218;

  fillRect(0, cy - 8, W, 16, ST77XX_BLACK);

  for (int i = 0; i < total; i++) {
    int cx = cx0 + i * gap;
    uint16_t col = (i == activeIndex) ? ST77XX_YELLOW : C_DARKGRAY;
    tft.fillCircle(cx, cy, dotR, col);
  }
}

// =============================================================
// Page 1 : Home page
// Shows live irradiance, tilt and azimuth.
// =============================================================
static void pageHome_static() {
  tft.fillScreen(ST77XX_BLACK);
  headerBar("IRRADIANCE");

  drawCard(10, 44, 220, 96, ST77XX_CYAN);
  drawSmallLabel(20, 54, "GLOBAL HORIZONTAL IRRADIANCE");

  drawCard(10, 150, 106, 60, ST77XX_CYAN);
  drawCard(124, 150, 106, 60, ST77XX_YELLOW);
  drawSmallLabel(20, 158, "TILT");
  drawSmallLabel(134, 158, "AZ");

  footerHint("Click: Next | Long: Cal");
  drawPageDots(0);

  ui.irrInt = -99999;
  ui.tilt10 = -999999;
  ui.az10   = -999999;
}

// Draw big irradiance value and icon
static void pageHome_drawIrr(int irrInt) {
  clearCardInner(10, 44, 220, 96);

  int kind = iconKindFromIrr((float)irrInt);

  int icx = 42;
  int icy = 92;

  if (kind == 0 || kind == 1) icx += 5;

  if (kind == 0)      drawIconCloudBig(icx, icy);
  else if (kind == 1) drawIconPartlyBig(icx, icy);
  else                drawIconSunBig(icx, icy);

  const GFXfont* f = &FreeSansBold24pt7b;

  char buf[10];
  snprintf(buf, sizeof(buf), "%d", irrInt);

  tft.setFont(f);
  tft.setTextSize(1);

  int16_t x1, y1;
  uint16_t tw, th;
  tft.getTextBounds(buf, 0, 0, &x1, &y1, &tw, &th);

  int baseX = 85;
  int baseY = 64;
  int areaW = 150;
  int areaH = 56;

  if (kind == 2) baseX -= 5;
  else           baseX += 5;

  if (irrInt >= 1000) baseX -= 5;

  int top = baseY + (areaH - (int)th) / 2;
  int baseline = top - y1;

  int startX = baseX - x1;

  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setCursor(startX, baseline);
  tft.print(buf);

  // Draw unit W/m2 with small superscript 2
  int unitGap = 8;
  int unitX = startX + tw + unitGap;
  int unitY = baseline + 4;

  tft.setFont(NULL);
  tft.setTextSize(2);
  tft.setTextColor(C_LIGHTGRAY, ST77XX_BLACK);
  tft.setCursor(unitX, unitY);
  tft.print("W/m");

  tft.setTextSize(1);
  tft.setCursor(unitX + 36, unitY - 8);
  tft.print("2");

  tft.setFont(NULL);
}

// Draw bottom tilt and azimuth values
static void pageHome_drawTiltAz() {
  fillRect(10 + CARD_INSET, 150 + 24, 106 - 2 * CARD_INSET, 60 - 30, ST77XX_BLACK);
  fillRect(124 + CARD_INSET, 150 + 24, 106 - 2 * CARD_INSET, 60 - 30, ST77XX_BLACK);

  char tbuf[16];
  char abuf[16];
  snprintf(tbuf, sizeof(tbuf), "%.1f", tiltDeg_disp);
  snprintf(abuf, sizeof(abuf), "%d", (int)lround(azimDeg_disp));

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

  tft.getTextBounds(tbuf, 0, 0, &x1, &y1, &w1, &h1);
  int cx = tx + (tw - (int)w1) / 2 - x1;
  int cyTop = ty + (th - (int)h1) / 2;
  int baseY = cyTop - y1 - 2;

  tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
  tft.setCursor(cx, baseY);
  tft.print(tbuf);

  int degX = cx + (int)w1 + 10;
  int degY = baseY - (int)h1 + 4;
  drawDegreeSymbol(degX, degY, ST77XX_CYAN);

  tft.getTextBounds(abuf, 0, 0, &x1, &y1, &w1, &h1);
  cx = ax + (aw - (int)w1) / 2 - x1;
  cyTop = ay + (ah - (int)h1) / 2;
  baseY = cyTop - y1 - 2;

  tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  tft.setCursor(cx, baseY);
  tft.print(abuf);

  degX = cx + (int)w1 + 8;
  degY = baseY - (int)h1 + 4;
  drawDegreeSymbol(degX, degY, ST77XX_YELLOW);

  tft.setFont(NULL);
}

// Dynamic updates for page 1
static void pageHome_dynamic() {
  headerUpdateStable(false);

  int irrInt = (int)lround(irr_filt);
  if (irrInt != ui.irrInt) {
    ui.irrInt = irrInt;
    pageHome_drawIrr(irrInt);
  }

  int tilt10 = (int)lround(tiltDeg_disp * 10.0f);
  int az10   = (int)lround(azimDeg_disp * 10.0f);
  if (tilt10 != ui.tilt10 || az10 != ui.az10) {
    ui.tilt10 = tilt10;
    ui.az10   = az10;
    pageHome_drawTiltAz();
  }
}

// =============================================================
// Page 2 : Min / Max page
// Shows live irradiance along with minimum and maximum values.
// =============================================================
static void pageMinMax_static() {
  tft.fillScreen(ST77XX_BLACK);
  headerBar("IRRADIANCE");

  drawCard(10, 44, 220, 96, ST77XX_YELLOW);
  drawSmallLabel(20, 54, "LIVE");

  drawCard(10, 150, 106, 60, ST77XX_CYAN);
  drawCard(124, 150, 106, 60, ST77XX_GREEN);
  drawSmallLabel(20, 160, "MIN");
  drawSmallLabel(134, 160, "MAX");

  footerHint("Click: Next");
  drawPageDots(1);

  ui.nowI = ui.minI = ui.maxI = -99999;
}

static void pageMinMax_dynamic() {
  headerUpdateStable(false);

  int nowI = (int)lround(irr_filt);
  int minI = isnan(irrMin) ? -1 : (int)lround(irrMin);
  int maxI = isnan(irrMax) ? -1 : (int)lround(irrMax);

  // Update live value
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

  // Small helper for MIN and MAX boxes
  auto drawSmallValueBox = [&](int x, int y, int w, int h, int value, uint16_t col, bool dash) {
    fillRect(x + CARD_INSET, y + 20, w - 2 * CARD_INSET, h - 24, ST77XX_BLACK);

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
    drawSmallValueBox(10, 150, 106, 60, minI, ST77XX_CYAN, (minI < 0));
  }
  if (maxI != ui.maxI) {
    ui.maxI = maxI;
    drawSmallValueBox(124, 150, 106, 60, maxI, ST77XX_GREEN, (maxI < 0));
  }
}

// =============================================================
// Page 3 : Tilt / Azimuth / Isc page
// Shows three stacked measurement boxes.
// =============================================================
static void pageTiltAz_static() {
  tft.fillScreen(ST77XX_BLACK);
  headerBar("TILT / AZ");

  int boxX = 10;
  int boxW = 220;

  int top = TOP_H + 10;
  int bottom = H - FOOT_H - 10;
  int availH = bottom - top;
  int boxH = (availH - 2 * 8) / 3;

  int y1 = top;
  int y2 = y1 + boxH + 8;
  int y3 = y2 + boxH + 8;

  drawCard(boxX, y1, boxW, boxH, ST77XX_CYAN);
  drawCard(boxX, y2, boxW, boxH, ST77XX_YELLOW);
  drawCard(boxX, y3, boxW, boxH, ST77XX_WHITE);

  drawSmallLabel(boxX + 10, y1 + 6, "TILT");
  drawSmallLabel(boxX + 10, y2 + 6, "AZIMUTH");
  drawSmallLabel(boxX + 10, y3 + 6, "ISC");

  footerHint("Click: Next");
  drawPageDots(2);

  ui.tilt10 = ui.az10 = ui.isc10 = -999999;
}

static void pageTiltAz_dynamic() {
  headerUpdateStable(false);

  int boxX = 10;
  int boxW = 220;

  int top = TOP_H + 10;
  int bottom = H - FOOT_H - 10;
  int availH = bottom - top;
  int boxH = (availH - 2 * 8) / 3;

  int y1 = top;
  int y2 = y1 + boxH + 8;
  int y3 = y2 + boxH + 8;

  // Reusable helper for large centered values
  auto drawCenteredValueWithOptionalSupDeg = [&](int x, int y, int w, int h,
                                                 const char* s,
                                                 const GFXfont* f,
                                                 uint16_t col,
                                                 bool drawSupDeg,
                                                 bool drawUnit_mA) {
    fillRect(x + CARD_INSET, y + 22, w - 2 * CARD_INSET, h - 26, ST77XX_BLACK);

    tft.setFont(f);
    tft.setTextSize(1);

    int16_t bx, by;
    uint16_t tw, th;
    tft.getTextBounds((char*)s, 0, 0, &bx, &by, &tw, &th);

    int cx = (x + CARD_INSET) + ((w - 2 * CARD_INSET) - (int)tw) / 2 - bx;
    int cy = (y + 22) + ((h - 26) - (int)th) / 2 - by - 8;

    tft.setTextColor(col, ST77XX_BLACK);
    tft.setCursor(cx, cy);
    tft.print(s);

    if (drawSupDeg) {
      int unitGap = 8;
      int degX = cx + (int)tw + unitGap;
      int degY = cy - (int)th + 4;
      drawDegreeSymbol(degX, degY, col);
    }

    if (drawUnit_mA) {
      int unitGap = 10;
      int unitX = cx + (int)tw + unitGap;
      int unitY = cy - 10;

      tft.setFont(NULL);
      tft.setTextSize(2);
      tft.setTextColor(C_LIGHTGRAY, ST77XX_BLACK);
      tft.setCursor(unitX, unitY);
      tft.print("mA");
      tft.setFont(NULL);
    }

    tft.setFont(NULL);
  };

  int tilt10 = (int)lround(tiltDeg_disp * 10.0f);
  if (tilt10 != ui.tilt10) {
    ui.tilt10 = tilt10;
    char b[16];
    snprintf(b, sizeof(b), "%.1f", tiltDeg_disp);
    drawCenteredValueWithOptionalSupDeg(boxX, y1, boxW, boxH,
                                        b, &FreeSansBold18pt7b, ST77XX_CYAN,
                                        true, false);
  }

  int az10 = (int)lround(azimDeg_disp * 10.0f);
  if (az10 != ui.az10) {
    ui.az10 = az10;
    char b[16];
    snprintf(b, sizeof(b), "%.1f", azimDeg_disp);
    drawCenteredValueWithOptionalSupDeg(boxX, y2, boxW, boxH,
                                        b, &FreeSansBold18pt7b, ST77XX_YELLOW,
                                        true, false);
  }

  int isc10 = (int)lround(iscmA_raw * 10.0f);
  if (isc10 != ui.isc10) {
    ui.isc10 = isc10;
    char b[16];
    snprintf(b, sizeof(b), "%.1f", iscmA_raw);
    drawCenteredValueWithOptionalSupDeg(boxX, y3, boxW, boxH,
                                        b, &FreeSansBold18pt7b, ST77XX_WHITE,
                                        false, true);
  }
}

// =============================================================
// Page 4 : Compass / Orientation page
// Shows azimuth value and a compass needle.
// =============================================================

// Convert azimuth degrees into compass direction text
static const char* azimuthToDirection(float deg) {
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

// Draw top box: Az value and direction text
static void drawAzValueBox(int x, int y, int w, int h, int azDeg, const char* dir) {
  fillRect(x, y, w, h, ST77XX_BLACK);

  char num[8];
  snprintf(num, sizeof(num), "%d", azDeg);

  const GFXfont* f = &FreeSansBold18pt7b;

  tft.setFont(NULL);
  tft.setTextSize(2);
  int16_t ax1, ay1;
  uint16_t atw, ath;
  tft.getTextBounds((char*)"Az:", 0, 0, &ax1, &ay1, &atw, &ath);

  tft.setFont(f);
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

  tft.setFont(f);
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

// Draw static compass circle, marks and labels
static void drawCompassRose(int cx, int cy, int r) {
  tft.drawCircle(cx, cy, r, C_LIGHTGRAY);
  tft.drawCircle(cx, cy, r - 1, C_LIGHTGRAY);
  tft.drawCircle(cx, cy, r - 2, C_DARKGRAY);

  tft.drawCircle(cx, cy, r - 16, C_DARKGRAY);

  for (int a = 0; a < 360; a += 10) {
    if (a == 0 || a == 90 || a == 180 || a == 270) continue;

    float rad = (a - 90) * (float)M_PI / 180.0f;

    int len;
    uint16_t col;

    if (a % 30 == 0) {
      len = 12;
      col = C_LIGHTGRAY;
    } else {
      len = 6;
      col = C_DARKGRAY;
    }

    int x1 = cx + (int)lround(cosf(rad) * (r - 3));
    int y1 = cy + (int)lround(sinf(rad) * (r - 3));
    int x2 = cx + (int)lround(cosf(rad) * (r - 3 - len));
    int y2 = cy + (int)lround(sinf(rad) * (r - 3 - len));

    tft.drawLine(x1, y1, x2, y2, col);

    if (a % 30 == 0) {
      tft.drawLine(x1 + 1, y1, x2 + 1, y2, col);
    }
  }

  tft.setFont(NULL);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

  int labelR = r - 14;

  tft.setCursor(cx - 6, cy - labelR - 8); tft.print("N");
  tft.setCursor(cx - 6, cy + labelR - 8); tft.print("S");
  tft.setCursor(cx - labelR - 6, cy - 8); tft.print("W");
  tft.setCursor(cx + labelR - 6, cy - 8); tft.print("E");

  tft.setFont(NULL);
}

// Draw compass needle
static void drawNeedle(int cx, int cy, int r, float deg) {
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
  tft.drawCircle(cx, cy, 9,  C_DARKGRAY);

  tft.fillCircle(cx, cy, 6, C_LIGHTGRAY);
  tft.drawCircle(cx, cy, 6, C_DARKGRAY);

  tft.fillCircle(cx, cy, 3, ST77XX_WHITE);
}

static void pageCompass_static() {
  tft.fillScreen(ST77XX_BLACK);
  headerBar("ORIENTATION");

  int valueY = TOP_H + 4;
  int valueH = 42;
  int compassY = valueY + valueH + 8;

  drawCard(10, valueY,   220, valueH, ST77XX_ORANGE);
  drawCard(10, compassY, 220, H - compassY - 8, ST77XX_ORANGE);

  ui.needleDeg = -999999;
}

static void pageCompass_dynamic() {
  headerUpdateStable(false);

  int azDeg = (int)lround(azimDeg_disp);
  azDeg = (azDeg % 360 + 360) % 360;

  if (azDeg == ui.needleDeg) return;
  ui.needleDeg = azDeg;

  int valueY = TOP_H + 4;
  int valueH = 42;
  int compassY = valueY + valueH + 8;

  clearCardInner(10, valueY, 220, valueH);

  const char* dir = azimuthToDirection(azimDeg_disp);

  int innerX = 10 + CARD_INSET;
  int innerY = valueY + CARD_INSET;
  int innerW = 220 - 2 * CARD_INSET;
  int innerH = valueH - 2 * CARD_INSET;

  drawAzValueBox(innerX, innerY, innerW, innerH, azDeg, dir);

  int boxX = 10;
  int boxY = compassY;
  int boxW = 220;
  int boxH = H - compassY - 8;

  const int COMPASS_INSET = 12;
  fillRect(boxX + COMPASS_INSET, boxY + COMPASS_INSET,
           boxW - 2 * COMPASS_INSET, boxH - 2 * COMPASS_INSET, ST77XX_BLACK);

  int safeX = boxX + COMPASS_INSET;
  int safeY = boxY + COMPASS_INSET;
  int safeW = boxW - 2 * COMPASS_INSET;
  int safeH = boxH - 2 * COMPASS_INSET;

  int cx = safeX + safeW / 2;
  int cy = safeY + safeH / 2;

  int r = min(safeW, safeH) / 2 + 2;
  if (r < 30) r = 30;

  drawCompassRose(cx, cy, r);
  drawNeedle(cx, cy, r, azimDeg_disp);
}

// =============================================================
// Page dispatcher
// These functions select which page to draw.
// =============================================================
static void resetUiCache() { ui = UiCache(); }

static void renderPageStatic(PageId p) {
  resetUiCache();
  if (p == PAGE_HOME) pageHome_static();
  else if (p == PAGE_MINMAX) pageMinMax_static();
  else if (p == PAGE_TILT_AZ) pageTiltAz_static();
  else pageCompass_static();
}

static void renderPageDynamic(PageId p) {
  if (p == PAGE_HOME) pageHome_dynamic();
  else if (p == PAGE_MINMAX) pageMinMax_dynamic();
  else if (p == PAGE_TILT_AZ) pageTiltAz_dynamic();
  else pageCompass_dynamic();
}

// =============================================================
// Calibration screen helper functions
// These are used by the step-by-step calibration wizard.
// =============================================================
static void calHeader(const char* title) {
  tft.fillScreen(ST77XX_BLACK);
  headerBar(title);
}

// Large step text like "STEP 1 OF 4"
static void calStepText(const char* s) {
  tft.setFont(NULL);
  tft.setTextSize(2);
  tft.setTextColor(C_LIGHTGRAY, ST77XX_BLACK);

  int16_t x1, y1;
  uint16_t tw, th;
  tft.getTextBounds((char*)s, 0, 0, &x1, &y1, &tw, &th);

  int x = (W - (int)tw) / 2 - x1;
  tft.setCursor(x, 46);
  tft.print(s);
}

// Title of the calibration step
static void calTitle(const char* s, uint16_t col) {
  tft.setFont(NULL);
  tft.setTextSize(2);
  tft.setTextColor(col, ST77XX_BLACK);

  int16_t x1, y1;
  uint16_t tw, th;
  tft.getTextBounds((char*)s, 0, 0, &x1, &y1, &tw, &th);

  int x = (W - (int)tw) / 2 - x1;
  tft.setCursor(x, 88);
  tft.print(s);
}

// Large centered instruction line
static void calInfoLine(const char* s, int y, uint16_t col = ST77XX_WHITE) {
  tft.setFont(NULL);
  tft.setTextSize(2);
  tft.setTextColor(col, ST77XX_BLACK);

  int16_t x1, y1;
  uint16_t tw, th;
  tft.getTextBounds((char*)s, 0, 0, &x1, &y1, &tw, &th);

  int x = (W - (int)tw) / 2 - x1;
  tft.setCursor(x, y);
  tft.print(s);
}

// Small note line, useful for extra hints
static void calSmallNote(const char* s, int y, uint16_t col = C_LIGHTGRAY) {
  tft.setFont(NULL);
  tft.setTextSize(1);
  tft.setTextColor(col, ST77XX_BLACK);

  int16_t x1, y1;
  uint16_t tw, th;
  tft.getTextBounds((char*)s, 0, 0, &x1, &y1, &tw, &th);

  int x = (W - (int)tw) / 2 - x1;
  tft.setCursor(x, y);
  tft.print(s);
}

// Footer not used during calibration to keep screen clean
static void calFooterHint(const char* s) {
  (void)s;
}

// Progress bar for compass calibration
static void calProgress(int pct, uint16_t col) {
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;

  int x = 20, y = 188, w = 200, h = 12;

  tft.drawRoundRect(x, y, w, h, 6, C_LIGHTGRAY);
  tft.fillRoundRect(x + 2, y + 2, w - 4, h - 4, 5, ST77XX_BLACK);

  int fw = (w - 6) * pct / 100;
  if (fw < 0) fw = 0;
  tft.fillRoundRect(x + 3, y + 3, fw, h - 6, 4, col);
}

// Show measured value after a calibration capture
static void calShowValue(const char* prefix, float value, const char* unit, uint16_t col) {
  char line[32];
  snprintf(line, sizeof(line), "%s%.2f %s", prefix, value, unit);

  fillRect(0, 150, W, 36, ST77XX_BLACK);
  calInfoLine(line, 170, col);
}

// Final screen after calibration is saved
static void calSavedAnimation() {
  calHeader("CALIBRATION");
  calStepText("DONE");
  calTitle("SAVED", ST77XX_GREEN);

  calInfoLine("Calibration Stored", 132, ST77XX_WHITE);
  calInfoLine("All Values Saved", 160, ST77XX_GREEN);

  for (int i = 0; i < 2; i++) {
    tft.drawCircle(120, 196, 10, ST77XX_GREEN);
    tft.drawLine(116, 196, 119, 200, ST77XX_GREEN);
    tft.drawLine(119, 200, 126, 190, ST77XX_GREEN);
    delay(140);

    fillRect(105, 184, 30, 30, ST77XX_BLACK);
    delay(90);
  }

  tft.drawCircle(120, 196, 10, ST77XX_GREEN);
  tft.drawLine(116, 196, 119, 200, ST77XX_GREEN);
  tft.drawLine(119, 200, 126, 190, ST77XX_GREEN);
  delay(1000);
}

// =============================================================
// Calibration button helpers
// A fresh press is required for every calibration step.
// =============================================================

// Wait until button is released
static void calWaitRelease() {
  while (digitalRead(PIN_BUTTON_GPIO) == LOW) {
    delay(10);
  }
  delay(120);
}

// Wait for a full new button press cycle
static void calWaitPress() {
  while (digitalRead(PIN_BUTTON_GPIO) == LOW) {
    delay(10);
  }
  delay(80);

  while (digitalRead(PIN_BUTTON_GPIO) == HIGH) {
    delay(10);
  }
  delay(30);

  while (digitalRead(PIN_BUTTON_GPIO) == LOW) {
    delay(10);
  }
  delay(120);
}

// =============================================================
// Calibration wizard
// Steps:
// 1. Measure dark Isc offset
// 2. Capture tilt zero position
// 3. Capture tilt 90 degree position
// 4. Capture magnetometer offsets
// =============================================================
static void runCalibrationQuick() {
  inCalibration = true;

  // -----------------------------------------------------------
  // STEP 1 : ISC OFFSET
  // User covers the panel so dark current can be measured.
  // -----------------------------------------------------------
  calHeader("CALIBRATION");
  calStepText("STEP 1 OF 4");
  calTitle("ISC OFFSET", ST77XX_YELLOW);
  calInfoLine("Cover the Panel", 128, ST77XX_WHITE);
  calInfoLine("Press Button to Save", 171, ST77XX_WHITE);

  calWaitPress();

#if DEMO_MODE
  iscOffset_mA = 1.2f;
  iscOffsetValid = true;
#else
  if (okIna) {
    float mA = measureIsc_mA_safe();
    if (mA < 0.0f) mA = 0.0f;
    iscOffset_mA = mA;
    iscOffsetValid = true;
  } else {
    iscOffset_mA = 0.0f;
    iscOffsetValid = false;
  }
#endif

  calShowValue("Isc Captured:", iscOffset_mA, "mA", ST77XX_GREEN);
  delay(1000);

  // -----------------------------------------------------------
  // STEP 2 : TILT ZERO
  // User keeps the device level and stores zero reference.
  // -----------------------------------------------------------
  calHeader("CALIBRATION");
  calStepText("STEP 2 OF 4");
  calTitle("TILT 0 DEG", ST77XX_CYAN);
  calInfoLine("Keep Device Level", 128, ST77XX_WHITE);
  calInfoLine("Press Button to Save", 171, ST77XX_WHITE);

  calWaitPress();

#if DEMO_MODE
  tiltPitch0 = 0.0f;
  tiltRoll0  = 0.0f;
#else
  if (okAcc) {
    sensors_event_t aEvent;
    accel.getEvent(&aEvent);

    float ax = aEvent.acceleration.x;
    float ay = aEvent.acceleration.y;
    float az = aEvent.acceleration.z;

    float an = sqrtf(ax * ax + ay * ay + az * az);
    if (an < 0.001f) an = 0.001f;

    float axn = ax / an;
    float ayn = ay / an;
    float azn = az / an;

    float roll  = atan2f(ayn, azn);
    float pitch = atan2f(-axn, sqrtf(ayn * ayn + azn * azn));

    tiltRoll0  = rad2deg(roll);
    tiltPitch0 = rad2deg(pitch);
  } else {
    tiltRoll0  = 0.0f;
    tiltPitch0 = 0.0f;
  }
#endif

  fillRect(0, 150, W, 36, ST77XX_BLACK);
  calInfoLine("0 Position Captured", 170, ST77XX_GREEN);
  delay(1000);

  // -----------------------------------------------------------
  // STEP 3 : TILT 90
  // User keeps the device vertical to calculate scale factor.
  // -----------------------------------------------------------
  calHeader("CALIBRATION");
  calStepText("STEP 3 OF 4");
  calTitle("TILT 90 DEG", ST77XX_CYAN);
  calInfoLine("Keep Device Vertical", 128, ST77XX_WHITE);
  calInfoLine("Press Button to Save", 171, ST77XX_WHITE);

  calWaitPress();

  float raw90 = 90.0f;

#if DEMO_MODE
  raw90 = 90.0f;
#else
  if (okAcc) {
    sensors_event_t aEvent;
    accel.getEvent(&aEvent);

    float ax = aEvent.acceleration.x;
    float ay = aEvent.acceleration.y;
    float az = aEvent.acceleration.z;

    float an = sqrtf(ax * ax + ay * ay + az * az);
    if (an < 0.001f) an = 0.001f;

    float axn = ax / an;
    float ayn = ay / an;
    float azn = az / an;

    float roll  = atan2f(ayn, azn);
    float pitch = atan2f(-axn, sqrtf(ayn * ayn + azn * azn));

    float rollDeg  = rad2deg(roll)  - tiltRoll0;
    float pitchDeg = rad2deg(pitch) - tiltPitch0;
    raw90 = sqrtf(rollDeg * rollDeg + pitchDeg * pitchDeg);
    if (raw90 < 1.0f) raw90 = 1.0f;
  } else {
    raw90 = 90.0f;
  }
#endif

  tiltScale = 90.0f / raw90;
  tiltCalValid = true;

  fillRect(0, 150, W, 36, ST77XX_BLACK);
  calInfoLine("90 Position Captured", 170, ST77XX_GREEN);
  delay(1000);

  // -----------------------------------------------------------
  // STEP 4 : COMPASS CALIBRATION
  // User rotates the device slowly so magnetic min/max values
  // can be collected.
  // -----------------------------------------------------------
  calHeader("CALIBRATION");
  calStepText("STEP 4 OF 4");
  calTitle("COMPASS", ST77XX_ORANGE);
  calInfoLine("Rotate Slowly", 128, ST77XX_WHITE);

  float minX =  1e9f, minY =  1e9f, minZ =  1e9f;
  float maxX = -1e9f, maxY = -1e9f, maxZ = -1e9f;

  unsigned long t0 = millis();
  const unsigned long DUR = 20000UL;

  while (millis() - t0 < DUR) {
    int pct = (int)((millis() - t0) * 100UL / DUR);
    calProgress(pct, ST77XX_ORANGE);

#if DEMO_MODE
    // In demo mode, no live sensor capture is needed.
#else
    if (okMag) {
      sensors_event_t mEvent;
      mag.getEvent(&mEvent);

      float mx = mEvent.magnetic.x;
      float my = mEvent.magnetic.y;
      float mz = mEvent.magnetic.z;

      if (mx < minX) minX = mx; if (mx > maxX) maxX = mx;
      if (my < minY) minY = my; if (my > maxY) maxY = my;
      if (mz < minZ) minZ = mz; if (mz > maxZ) maxZ = mz;
    }
#endif
    delay(40);
  }

#if DEMO_MODE
  magOffsetX = magOffsetY = magOffsetZ = 0.0f;
  magCalValid = true;
#else
  if (okMag) {
    magOffsetX = 0.5f * (maxX + minX);
    magOffsetY = 0.5f * (maxY + minY);
    magOffsetZ = 0.5f * (maxZ + minZ);
    magCalValid = true;
  } else {
    magOffsetX = magOffsetY = magOffsetZ = 0.0f;
    magCalValid = false;
  }
#endif

  saveCalToEEPROM();
  calSavedAnimation();

  inCalibration = false;
  renderPageStatic(PAGE_SEQ[seqIndex]);
}

// =============================================================
// Button handling
// Short press = next page
// Long press = run calibration
// =============================================================
static void gotoNextPage() {
  seqIndex = (seqIndex + 1) % 4;
  renderPageStatic(PAGE_SEQ[seqIndex]);
}

static void handleButton() {
  unsigned long now = millis();
  bool raw = digitalRead(PIN_BUTTON_GPIO);

  static bool lastRaw = true;
  static unsigned long lastChange = 0;

  // Track raw edge changes for debounce
  if (raw != lastRaw) {
    lastRaw = raw;
    lastChange = now;
  }
  if (now - lastChange < DEBOUNCE_MS) return;

  if (inCalibration) return;

  // Button pressed
  if (raw == LOW) {
    if (btnDownMs == 0) {
      btnDownMs = now;
      longPressFired = false;
    } else if (!longPressFired && (now - btnDownMs >= LONG_PRESS_MS)) {
      longPressFired = true;
      runCalibrationQuick();
    }
  }
  // Button released
  else {
    if (btnDownMs != 0) {
      unsigned long pressMs = now - btnDownMs;
      btnDownMs = 0;
      if (!longPressFired && pressMs < LONG_PRESS_MS) gotoNextPage();
    }
  }
}

// =============================================================
// Arduino setup
// Runs once at power up.
// =============================================================
void setup() {
#if ENABLE_SERIAL_DEBUG
  Serial.begin(115200);
  delay(250);
  Serial.println();
  Serial.println("Solar Irradiance Meter - Professional LCD UI");
#endif

  // Button uses internal pullup
  pinMode(PIN_BUTTON_GPIO, INPUT_PULLUP);

  // LCD backlight output
  pinMode(LCD_BL, OUTPUT);
  analogWrite(LCD_BL, 0);

  // MOSFET is OFF by default
  pinMode(PIN_MOSFET_GPIO, OUTPUT);
  digitalWrite(PIN_MOSFET_GPIO, LOW);

  analogReadResolution(12);

  // Start I2C and SPI
  Wire.begin();
  Wire.setClock(400000);

  SPI.begin(LCD_SCK, -1, LCD_MOSI, LCD_CS);
  tft.init(240, 240);
  tft.setRotation(3);
  tft.setSPISpeed(40000000);

  // Show startup screen first
  showBootScreen();

  // Start EEPROM and load any saved calibration
  EEPROM.begin(EEPROM_BYTES);
  loadCalFromEEPROM();

#if !DEMO_MODE
  okIna = ina.begin();
  okAcc = accel.begin();
  okMag = mag.begin();
#else
  okIna = false;
  okAcc = false;
  okMag = false;
#endif

#if DEMO_MODE
  // Fixed demo values for UI testing
  irr_filt     = 892.0f;
  irrMin       = 220.0f;
  irrMax       = 980.0f;
  tiltDeg_disp = 17.5f;
  azimDeg_disp = 238.0f;
  iscmA_raw    = 142.8f;
  battPct      = 78.0f;
#else
  // Get initial live readings
  vbat = readBatteryVoltage();
  battPct = batteryPercentFromVoltageCurve(vbat);

  updateTiltAndAzimuth();

  if (okIna) {
    iscmA_raw = measureIsc_mA_safe();

    float offset = iscOffsetValid ? iscOffset_mA : 0.0f;
    float iscCorr = iscmA_raw - offset;
    if (iscCorr < 0.0f) iscCorr = 0.0f;

    irr_now  = iscCorr * CAL_K_WM2_PER_MA;
    irr_filt = irr_now;
    irrMin   = irr_filt;
    irrMax   = irr_filt;
  } else {
    irr_now  = 0.0f;
    irr_filt = 0.0f;
    irrMin   = NAN;
    irrMax   = NAN;
  }
#endif

  // Start from first page
  seqIndex = 0;
  renderPageStatic(PAGE_SEQ[seqIndex]);

  // Initialize software timers
  unsigned long now = millis();
  lastMeasMs = now;
  lastImuMs  = now;
  lastBatMs  = now;
  lastUiMs   = now;
}

// =============================================================
// Arduino main loop
// Runs repeatedly.
// =============================================================
void loop() {
  unsigned long now = millis();

  handleButton();
  if (inCalibration) {
    delay(1);
    return;
  }

#if DEMO_MODE
  // Demo values can be changed here for screen testing
  irr_filt     = 200.0f;
  irrMin       = 220.0f;
  irrMax       = 980.0f;
  tiltDeg_disp = 17.5f;
  azimDeg_disp = 215.0f;
  iscmA_raw    = 142.8f;
  battPct      = 55.0f;

  if (now - lastBatMs >= BAT_PERIOD_MS) {
    lastBatMs = now;
    headerUpdateStable(false);
  }
#else
  // Battery update
  if (now - lastBatMs >= BAT_PERIOD_MS) {
    vbat = readBatteryVoltage();
    battPct = batteryPercentFromVoltageCurve(vbat);
    lastBatMs = now;
    headerUpdateStable(false);
  }

  // IMU update
  if (now - lastImuMs >= IMU_PERIOD_MS) {
    updateTiltAndAzimuth();
    lastImuMs = now;
  }

  // Irradiance update
  if (now - lastMeasMs >= MEAS_PERIOD_MS) {
    updateIrradiance();
    lastMeasMs = now;
  }
#endif

  // UI refresh
  if (now - lastUiMs >= UI_PERIOD_MS) {
    renderPageDynamic(PAGE_SEQ[seqIndex]);
    lastUiMs = now;
  }

#if ENABLE_SERIAL_DEBUG
  static unsigned long lastS = 0;
  if (now - lastS > 800) {
    lastS = now;
    Serial.print("Irr=");
    Serial.print(irr_filt, 0);
    Serial.print(" W/m2 | Tilt=");
    Serial.print(tiltDeg_disp, 1);
    Serial.print(" | Az=");
    Serial.print(azimDeg_disp, 1);
    Serial.print(" | Bat=");
    Serial.print(battPct, 0);
    Serial.println("%");
  }
#endif

  delay(1);
}
