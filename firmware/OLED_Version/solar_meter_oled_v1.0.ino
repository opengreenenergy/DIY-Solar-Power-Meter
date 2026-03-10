/*
===============================================================================
  PROJECT       : DIY Solar Power Meter
  VERSION       : v1.0
  UPDATED ON    : 15-Feb-2026
  AUTHOR        : Open Green Energy

  LICENSE
  ---------------------------------------------------------------------------
  Copyright (c) 2026 Open Green Energy

  This firmware is shared as open source for learning, personal use,
  educational use, and research purposes.

  Commercial use, commercial redistribution, commercial manufacturing, or use
  in commercial products by third parties is not permitted without prior
  written permission from Open Green Energy.

  If you plan to publish, modify, or share this work, proper credit to
  Open Green Energy should be given.

  PROJECT OVERVIEW
  ---------------------------------------------------------------------------
  This is the OLED display version of the DIY Solar Power Meter.

  It measures solar irradiance by reading short circuit current from a
  reference solar panel through INA226. It also displays:

  - Irradiance
  - Tilt angle
  - Azimuth / heading
  - Battery percentage
  - Compass page

  Calibration values are stored in EEPROM so they remain available after
  power off.

  KEY FEATURES
  ---------------------------------------------------------------------------
  - Uses XIAO ESP32-C3 Arduino pin names: D0, D1, D2
  - Responsive single button control
  - Long press starts calibration
  - Calibration always waits for a fresh button click
  - Normal UI pages are locked during calibration
  - EEPROM based calibration storage
  - Separate pages for Home, Min/Max, Tilt/Azimuth, and Compass

  DISPLAY PAGES
  ---------------------------------------------------------------------------
  Page 1 : HOME
           Battery + Irradiance + Tilt

  Page 2 : MIN / MAX
           Live Irradiance + Minimum + Maximum

  Page 3 : TILT / AZIMUTH
           Tilt + Azimuth

  Page 4 : COMPASS
           Compass circle + needle + azimuth readout

  PIN MAPPING (XIAO ESP32-C3)
  ---------------------------------------------------------------------------
  PIN_BUTTON_GPIO   : D2         (INPUT_PULLUP) -> Single push button
  PIN_MOSFET_GPIO   : D1         (OUTPUT)       -> MOSFET gate for Isc short
  PIN_BAT_SENSE     : D0 / A0    (ADC)          -> Battery voltage sense
  I2C BUS           : SDA / SCL                 -> OLED + INA226 + IMU

  SENSOR DETAILS
  ---------------------------------------------------------------------------
  1) INA226
     Measures short circuit current (Isc) in mA
     Irradiance is calculated using a calibration factor

  2) LIS2MDL + LSM303 Accelerometer
     Accelerometer gives tilt
     Magnetometer gives azimuth / heading

  3) Battery ADC
     Reads battery voltage through resistor divider
     Converts battery voltage into percentage

  REVISION NOTES
  ---------------------------------------------------------------------------
  1. Uses D0, D1, D2 pin names instead of GPIO numbers
  2. Button handling is improved for better response
  3. Calibration always requires a fresh button click
  4. During calibration, normal page rendering is disabled

===============================================================================
*/

#include <Wire.h>
#include <math.h>
#include <EEPROM.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_LSM303_Accel.h>

#include <INA226.h>

// ======================================================
// User settings
// Change these values only if you know the hardware setup.
// ======================================================
#define ENABLE_SERIAL_DEBUG  1

// ======================================================
// Pin mapping
// Using Seeed XIAO ESP32-C3 Arduino pin names
// ======================================================
static const int PIN_BAT_SENSE   = D0;  // Battery voltage ADC input
static const int PIN_MOSFET_GPIO = D1;  // MOSFET gate control
static const int PIN_BUTTON_GPIO = D2;  // Single push button

// ======================================================
// INA226 settings
// Used for measuring short circuit current from panel
// ======================================================
static const uint8_t INA_ADDR = 0x44;
static const float   SHUNT_OHMS = 0.50f;
static const float   CAL_K_WM2_PER_MA = 15.05f;

// ======================================================
// Battery divider
// Used to scale battery voltage to ADC range
// ======================================================
static const float BAT_RTOP = 200000.0f;
static const float BAT_RBOT = 100000.0f;

// ======================================================
// Battery model
// Used for converting voltage to battery percentage
// ======================================================
static const float VBAT_FULL  = 4.18f;
static const float VBAT_EMPTY = 3.30f;

// ======================================================
// Compass / heading configuration
// ======================================================
static const float DECLINATION_DEG = 0.0f;
static const float TILT_REF_DEG = 90.0f;
static const float HEADING_OFFSET_DEG = 0.0f;
static const bool  HEADING_IS_NORTH = true;

// ======================================================
// OLED display settings
// ======================================================
#define OLED_W 128
#define OLED_H 64
#define OLED_RST -1
#define OLED_ADDR 0x3C

static const int TOP_UI_SHIFT_Y = 3;
static const char DEG_SYM = (char)247;

// ======================================================
// ADC and timing settings
// ======================================================
static const float ADC_VREF = 3.3f;
static const unsigned int ADC_MAX = 4095;

static const unsigned long UI_REFRESH_MS  = 120;
static const unsigned long MEAS_PERIOD_MS = 800;
static const unsigned long IMU_PERIOD_MS  = 120;
static const unsigned long BAT_PERIOD_MS  = 1000;

// Button timing
static const unsigned long DEBOUNCE_MS    = 18;
static const unsigned long LONG_PRESS_MS  = 2000;

// Safe INA226 Isc sampling timing
static const unsigned int ISC_SETTLE_MS     = 200;
static const uint8_t      ISC_SAMPLES       = 20;
static const unsigned int ISC_SAMPLE_GAP_MS = 10;

// Display smoothing
static const float IRR_ALPHA  = 0.25f;
static const float TILT_ALPHA = 0.45f;
static const float AZ_ALPHA   = 0.35f;

// ======================================================
// EEPROM settings
// Calibration values are stored here
// ======================================================
static const int EEPROM_BYTES = 128;
static const uint32_t CAL_MAGIC = 0xA11C0A1Bu;
static const uint16_t CAL_VER   = 6;

// Structure used to store all calibration values
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

  uint8_t  pad[16];
};

// ======================================================
// Object creation
// ======================================================
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, OLED_RST);

INA226 ina(INA_ADDR);
Adafruit_LIS2MDL mag;
Adafruit_LSM303_Accel_Unified accel(54321);

// ======================================================
// Page control
// ======================================================
enum PageId : uint8_t { PAGE_HOME, PAGE_MINMAX, PAGE_TILT_AZ, PAGE_COMPASS };
static const PageId PAGE_SEQ[4] = { PAGE_HOME, PAGE_MINMAX, PAGE_TILT_AZ, PAGE_COMPASS };
uint8_t seqIndex = 0;

// ======================================================
// Runtime state
// These variables hold live measurement values
// ======================================================
bool okIna = false;
bool okAcc = false;
bool okMag = false;

float vbat = 0.0f;
float battPct = 0.0f;

float tiltDeg_raw  = 0.0f;
float tiltDeg_disp = 0.0f;

float azimDeg_raw  = 0.0f;
float azimDeg_disp = 0.0f;

float iscmA_raw = 0.0f;
float irr_now   = 0.0f;
float irr_filt  = 0.0f;

float irrMin = NAN;
float irrMax = NAN;

unsigned long lastUiMs   = 0;
unsigned long lastMeasMs = 0;
unsigned long lastImuMs  = 0;
unsigned long lastBatMs  = 0;

// Button state
bool btnLast = true;
unsigned long btnLastChangeMs = 0;
unsigned long btnDownMs = 0;
bool longPressFired = false;

// Calibration lock flag
bool inCalibration = false;

// Calibration values currently loaded in RAM
float iscOffset_mA = 0.0f;
bool  iscOffsetValid = false;

float tiltPitch0 = 0.0f;
float tiltRoll0  = 0.0f;
float tiltScale  = 1.0f;
bool  tiltCalValid = false;

float magOffsetX = 0.0f;
float magOffsetY = 0.0f;
float magOffsetZ = 0.0f;
bool  magCalValid = false;

// Forward declarations
static void handleButton();
static void runFullCalibrationAndSave();
static void renderPage(PageId p);

// ======================================================
// Math helper functions
// ======================================================
static float rad2deg(float r) { return r * 180.0f / (float)M_PI; }

static float wrap360(float deg) {
  while (deg < 0) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  return deg;
}

static float wrap180(float deg) {
  while (deg < -180.0f) deg += 360.0f;
  while (deg >= 180.0f) deg -= 360.0f;
  return deg;
}

static float clampf(float x, float a, float b) {
  if (x < a) return a;
  if (x > b) return b;
  return x;
}

static float mapf(float x, float inMin, float inMax, float outMin, float outMax) {
  if ((inMax - inMin) == 0.0f) return outMin;
  float t = (x - inMin) / (inMax - inMin);
  return outMin + t * (outMax - outMin);
}

// Converts azimuth to final heading based on offset and north/south choice
static float headingFromAzimuth(float azDeg) {
  float h = azDeg;
  if (!HEADING_IS_NORTH) h += 180.0f;
  h += HEADING_OFFSET_DEG;
  return wrap360(h);
}

// ======================================================
// Battery helper functions
// ======================================================
static float batteryPercentFromVoltage(float v) {
  float p = mapf(v, VBAT_EMPTY, VBAT_FULL, 0.0f, 100.0f);
  return clampf(p, 0.0f, 100.0f);
}

// Read battery voltage through ADC and resistor divider
static float readBatteryVoltage() {
  unsigned int raw = analogRead(PIN_BAT_SENSE);
  float vadc = (raw * ADC_VREF) / (float)ADC_MAX;
  float vb = vadc * (BAT_RTOP + BAT_RBOT) / BAT_RBOT;
  return vb;
}

// ======================================================
// UI helper functions
// ======================================================

// Print text centered horizontally
static void printCentered(int y, const char* txt, uint8_t size) {
  int16_t x1, y1;
  uint16_t w, h;
  display.setTextSize(size);
  display.getTextBounds(txt, 0, y, &x1, &y1, &w, &h);
  display.setCursor((OLED_W - (int)w) / 2, y);
  display.print(txt);
}

// Draw battery icon and percentage on top right
static void drawBatteryTopRight(int percent) {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  percent = (int)clampf((float)percent, 0.0f, 100.0f);

  const int bw = 22, bh = 10;
  const int bx = OLED_W - bw - 2;
  const int by = 1 + TOP_UI_SHIFT_Y;

  display.drawRect(bx, by, bw, bh, SSD1306_WHITE);
  display.fillRect(bx + bw, by + 2, 3, bh - 4, SSD1306_WHITE);

  int fillW = (bw - 4) * percent / 100;
  display.fillRect(bx + 2, by + 2, fillW, bh - 4, SSD1306_WHITE);

  display.setCursor(bx - 26, TOP_UI_SHIFT_Y);
  display.print(percent);
  display.print("%");
}

// Draw a simple 3-line prompt screen used during calibration
static void drawPrompt3(const char* l1, const char* l2, const char* l3) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  printCentered(0,  l1, 1);
  printCentered(20, l2, 2);
  printCentered(50, l3, 1);
  display.display();
}

// Power on splash screen
static void showSplash() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  printCentered(18, "SOLAR POWR", 2);
  printCentered(42, "METER", 2);
  display.display();
  delay(700);
}

// ======================================================
// Wait helpers
// Used in safe sampling and calibration steps
// ======================================================

// Allows short background handling while waiting
static void serviceDuringWait(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    handleButton();   // ignored when inCalibration is true
    delay(1);
  }
}

// Requires a true fresh click sequence:
// released -> press -> released
static void waitForFreshClick() {
  while (digitalRead(PIN_BUTTON_GPIO) == LOW) delay(1);
  delay(DEBOUNCE_MS);

  while (digitalRead(PIN_BUTTON_GPIO) == HIGH) delay(1);
  delay(DEBOUNCE_MS);

  while (digitalRead(PIN_BUTTON_GPIO) == LOW) delay(1);
  delay(160);
}

// ======================================================
// EEPROM load / save functions
// ======================================================
static void loadCalFromEEPROM() {
  CalData cd;
  EEPROM.get(0, cd);

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
  if (!isfinite(tiltScale) || tiltScale <= 0.1f || tiltScale > 10.0f) {
    tiltScale = 1.0f;
    tiltCalValid = false;
  }

  magCalValid = (cd.magValid != 0);
  magOffsetX  = cd.magOffX;
  magOffsetY  = cd.magOffY;
  magOffsetZ  = cd.magOffZ;
}

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

// ======================================================
// INA226 and irradiance functions
// ======================================================

// Measure Isc by briefly turning on the MOSFET and averaging samples
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

// Update irradiance from current measurement and stored dark offset
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

// ======================================================
// IMU tilt and azimuth functions
// ======================================================
static void updateTiltAndAzimuth() {
  float roll = 0.0f, pitch = 0.0f;

  // Tilt calculation using accelerometer
  if (okAcc) {
    sensors_event_t aEvent;
    accel.getEvent(&aEvent);

    float ax = aEvent.acceleration.x;
    float ay = aEvent.acceleration.y;
    float az = aEvent.acceleration.z;

    float an = sqrtf(ax*ax + ay*ay + az*az);
    if (an < 0.001f) an = 0.001f;

    float axn = ax / an;
    float ayn = ay / an;
    float azn = az / an;

    roll  = atan2f(ayn, azn);
    pitch = atan2f(-axn, sqrtf(ayn*ayn + azn*azn));

    float rollDeg  = rad2deg(roll)  - tiltRoll0;
    float pitchDeg = rad2deg(pitch) - tiltPitch0;
    float rawMag = sqrtf(rollDeg*rollDeg + pitchDeg*pitchDeg);

    tiltDeg_raw  = tiltCalValid ? (rawMag * tiltScale) : rawMag;
    tiltDeg_disp = tiltDeg_disp + TILT_ALPHA * (tiltDeg_raw - tiltDeg_disp);
  } else {
    tiltDeg_raw = tiltDeg_disp = 0.0f;
  }

  // Azimuth calculation using magnetometer with tilt compensation
  if (okMag) {
    sensors_event_t mEvent;
    mag.getEvent(&mEvent);

    float mx = mEvent.magnetic.x - (magCalValid ? magOffsetX : 0.0f);
    float my = mEvent.magnetic.y - (magCalValid ? magOffsetY : 0.0f);
    float mz = mEvent.magnetic.z - (magCalValid ? magOffsetZ : 0.0f);

    float cosR = cosf(roll),  sinR = sinf(roll);
    float cosP = cosf(pitch), sinP = sinf(pitch);

    float Xh = mx * cosP + mz * sinP;
    float Yh = mx * sinR * sinP + my * cosR - mz * sinR * cosP;

    float azimuth = rad2deg(atan2f(-Xh, Yh));
    azimuth = wrap360(azimuth + 180.0f);
    azimDeg_raw = wrap360(azimuth + DECLINATION_DEG);

    float d = wrap180(azimDeg_raw - azimDeg_disp);
    azimDeg_disp = wrap360(azimDeg_disp + AZ_ALPHA * d);
  } else {
    azimDeg_raw = azimDeg_disp = 0.0f;
  }
}

// ======================================================
// Page drawing functions
// ======================================================

// Page 1: Home page
static void drawHomePage() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  drawBatteryTopRight((int)lround(battPct));

  char buf[24];
  snprintf(buf, sizeof(buf), "%d W/m2", (int)lround(irr_filt));
  printCentered(18, buf, 2);

  display.drawLine(10, 36, 118, 36, SSD1306_WHITE);

  snprintf(buf, sizeof(buf), "%0.1f%c", tiltDeg_disp, DEG_SYM);
  printCentered(46, buf, 1);

  display.display();
}

// Page 2: Min / Max page
static void drawMinMaxPage() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  printCentered(2 + TOP_UI_SHIFT_Y, "MIN / MAX", 1);

  char buf[24];
  snprintf(buf, sizeof(buf), "%d W/m2", (int)lround(irr_filt));
  printCentered(18, buf, 2);

  display.drawLine(8, 34, OLED_W - 9, 34, SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(2, 42);
  display.print("Min: ");
  if (isnan(irrMin)) display.print("--");
  else display.print((int)lround(irrMin));

  display.setCursor(64, 42);
  display.print("Max: ");
  if (isnan(irrMax)) display.print("--");
  else display.print((int)lround(irrMax));

  display.display();
}

// Page 3: Tilt / Azimuth page
static void drawTiltAzPage() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  printCentered(2 + TOP_UI_SHIFT_Y, "TILT / AZIMUTH", 1);

  char topLine[20];
  char botLine[20];
  snprintf(topLine, sizeof(topLine), "%0.1f%c", tiltDeg_disp, DEG_SYM);
  snprintf(botLine, sizeof(botLine), "%0.1f%c", azimDeg_disp, DEG_SYM);

  printCentered(20, topLine, 2);
  display.drawLine(8, 38, OLED_W - 9, 38, SSD1306_WHITE);
  printCentered(44, botLine, 2);

  display.display();
}

// Page 4: Compass page
static void drawCompassOnlyPage() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(5, 3);
  display.print("Az:");
  display.print((int)lroundf(azimDeg_disp));
  display.write(DEG_SYM);

  const int cx = 64;
  const int cy = 34;
  const int r  = 23;

  display.drawCircle(cx, cy, r, SSD1306_WHITE);

  for (int deg = 0; deg < 360; deg += 30) {
    float a = deg * (float)M_PI / 180.0f;
    const int len = 6;

    int x0 = cx + (int)lroundf(sinf(a) * (r - 1));
    int y0 = cy - (int)lroundf(cosf(a) * (r - 1));
    int x1 = cx + (int)lroundf(sinf(a) * (r - 1 - len));
    int y1 = cy - (int)lroundf(cosf(a) * (r - 1 - len));

    display.drawLine(x0, y0, x1, y1, SSD1306_WHITE);
  }

  // Note: compass labels are drawn according to the project’s chosen orientation
  display.setTextSize(1);
  display.setCursor(cx - 3,      cy - r - 9); display.print("S");
  display.setCursor(cx + r + 4,  cy - 3);     display.print("W");
  display.setCursor(cx - 3,      cy + r + 2); display.print("N");
  display.setCursor(cx - r - 10, cy - 3);     display.print("E");

  float headingNorth = headingFromAzimuth(azimDeg_disp);
  float displayAngle = wrap360(headingNorth + 180.0f);
  float a = displayAngle * (float)M_PI / 180.0f;

  int xTip  = cx + (int)lroundf(sinf(a) * (r - 6));
  int yTip  = cy - (int)lroundf(cosf(a) * (r - 6));
  int xTail = cx - (int)lroundf(sinf(a) * (r - 11));
  int yTail = cy + (int)lroundf(cosf(a) * (r - 11));

  display.drawLine(xTail, yTail, xTip, yTip, SSD1306_WHITE);

  float ap = a + (float)M_PI / 2.0f;
  const int w = 4;

  int xL = xTip - (int)lroundf(sinf(a) * 2) + (int)lroundf(sinf(ap) * w);
  int yL = yTip + (int)lroundf(cosf(a) * 2) - (int)lroundf(cosf(ap) * w);
  int xR = xTip - (int)lroundf(sinf(a) * 2) - (int)lroundf(sinf(ap) * w);
  int yR = yTip + (int)lroundf(cosf(a) * 2) + (int)lroundf(cosf(ap) * w);

  display.fillTriangle(xTip, yTip, xL, yL, xR, yR, SSD1306_WHITE);
  display.fillCircle(cx, cy, 2, SSD1306_WHITE);

  display.display();
}

// Dispatch page rendering
static void renderPage(PageId p) {
  if (p == PAGE_HOME)         drawHomePage();
  else if (p == PAGE_MINMAX)  drawMinMaxPage();
  else if (p == PAGE_TILT_AZ) drawTiltAzPage();
  else                        drawCompassOnlyPage();
}

// ======================================================
// Calibration helper functions
// ======================================================

// Average pitch or roll over multiple samples
static float avgPitchOrRoll(bool wantPitch, int samples) {
  float sum = 0.0f;
  for (int i = 0; i < samples; i++) {
    sensors_event_t aEvent;
    accel.getEvent(&aEvent);

    float ax = aEvent.acceleration.x;
    float ay = aEvent.acceleration.y;
    float az = aEvent.acceleration.z;

    float an = sqrtf(ax*ax + ay*ay + az*az);
    if (an < 0.001f) an = 0.001f;

    float axn = ax / an;
    float ayn = ay / an;
    float azn = az / an;

    float roll  = atan2f(ayn, azn);
    float pitch = atan2f(-axn, sqrtf(ayn*ayn + azn*azn));

    sum += wantPitch ? rad2deg(pitch) : rad2deg(roll);
    serviceDuringWait(25);
  }
  return sum / (float)samples;
}

// Average raw tilt magnitude used for 90 degree reference
static float avgRawTiltMagnitude(int samples) {
  float sum = 0.0f;
  for (int i = 0; i < samples; i++) {
    sensors_event_t aEvent;
    accel.getEvent(&aEvent);

    float ax = aEvent.acceleration.x;
    float ay = aEvent.acceleration.y;
    float az = aEvent.acceleration.z;

    float an = sqrtf(ax*ax + ay*ay + az*az);
    if (an < 0.001f) an = 0.001f;

    float axn = ax / an;
    float ayn = ay / an;
    float azn = az / an;

    float rollDeg  = rad2deg(atan2f(ayn, azn)) - tiltRoll0;
    float pitchDeg = rad2deg(atan2f(-axn, sqrtf(ayn*ayn + azn*azn))) - tiltPitch0;
    sum += sqrtf(rollDeg*rollDeg + pitchDeg*pitchDeg);

    serviceDuringWait(25);
  }
  return sum / (float)samples;
}

// Step 1: dark offset calibration for irradiance
static void calibrateIrradianceDarkOffset_wait() {
  if (!okIna) return;

  drawPrompt3("CAL IRR", "COVER PANL", "Press Btn");
  waitForFreshClick();

  float mA = measureIsc_mA_safe();
  if (mA < 0.0f) mA = 0.0f;

  iscOffset_mA = mA;
  iscOffsetValid = true;
}

// Step 2 and 3: two point tilt calibration
static void calibrateTiltTwoPoint_wait() {
  if (!okAcc) return;

  char z0[10];  snprintf(z0, sizeof(z0), "0%c",  DEG_SYM);
  char z90[10]; snprintf(z90, sizeof(z90), "90%c", DEG_SYM);

  drawPrompt3("CAL TILT", z0, "Press Btn");
  waitForFreshClick();

  const int N = 25;
  tiltPitch0 = avgPitchOrRoll(true, N);
  tiltRoll0  = avgPitchOrRoll(false, N);

  drawPrompt3("CAL TILT", z90, "Press Btn");
  waitForFreshClick();

  float rawKnown = avgRawTiltMagnitude(N);
  if (rawKnown < 1.0f) rawKnown = 1.0f;

  tiltScale = TILT_REF_DEG / rawKnown;
  tiltCalValid = true;
}

// Step 4: rotate device to capture magnetometer offsets
static void calibrateMagOffsetsRotate_timer(uint16_t seconds) {
  if (!okMag) return;

  drawPrompt3("CAL AZIM", "ROTATE", "Press Btn");
  waitForFreshClick();

  float minX =  1e9f, minY =  1e9f, minZ =  1e9f;
  float maxX = -1e9f, maxY = -1e9f, maxZ = -1e9f;

  unsigned long t0 = millis();
  unsigned long durationMs = (unsigned long)seconds * 1000UL;

  while (millis() - t0 < durationMs) {
    sensors_event_t mEvent;
    mag.getEvent(&mEvent);

    float mx = mEvent.magnetic.x;
    float my = mEvent.magnetic.y;
    float mz = mEvent.magnetic.z;

    if (mx < minX) minX = mx;
    if (my < minY) minY = my;
    if (mz < minZ) minZ = mz;

    if (mx > maxX) maxX = mx;
    if (my > maxY) maxY = my;
    if (mz > maxZ) maxZ = mz;

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    printCentered(0, "CAL AZIM", 1);
    printCentered(18, "ROTATE", 2);

    float frac = (float)(millis() - t0) / (float)durationMs;
    frac = clampf(frac, 0.0f, 1.0f);

    int barW = (int)lround(frac * 120.0f);
    display.drawRect(4, 56, 120, 6, SSD1306_WHITE);
    int fillW = barW - 2; if (fillW < 0) fillW = 0;
    display.fillRect(5, 57, fillW, 4, SSD1306_WHITE);

    display.display();
    serviceDuringWait(25);
  }

  magOffsetX = (maxX + minX) * 0.5f;
  magOffsetY = (maxY + minY) * 0.5f;
  magOffsetZ = (maxZ + minZ) * 0.5f;
  magCalValid = true;
}

// Full calibration sequence with EEPROM save
static void runFullCalibrationAndSave() {
  inCalibration = true;

  while (digitalRead(PIN_BUTTON_GPIO) == LOW) delay(1);
  delay(80);

  calibrateIrradianceDarkOffset_wait();
  calibrateTiltTwoPoint_wait();
  calibrateMagOffsetsRotate_timer(20);

  saveCalToEEPROM();

  updateTiltAndAzimuth();
  tiltDeg_disp = tiltDeg_raw;
  azimDeg_disp = azimDeg_raw;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  printCentered(22, "SAVED", 2);
  display.display();
  delay(650);

  // Reset button state so page does not accidentally change after calibration
  btnDownMs = 0;
  longPressFired = false;
  btnLast = digitalRead(PIN_BUTTON_GPIO);
  btnLastChangeMs = millis();

  inCalibration = false;
}

// ======================================================
// Button handling
// Short press = next page
// Long press = full calibration
// ======================================================
static void handleButton() {
  unsigned long now = millis();
  bool raw = digitalRead(PIN_BUTTON_GPIO);

  // Ignore page actions while calibration is running
  if (inCalibration) {
    btnLast = raw;
    btnLastChangeMs = now;
    btnDownMs = 0;
    longPressFired = true;
    return;
  }

  if (raw != btnLast) {
    btnLast = raw;
    btnLastChangeMs = now;
    return;
  }
  if (now - btnLastChangeMs < DEBOUNCE_MS) return;

  if (raw == LOW) {
    if (btnDownMs == 0) {
      btnDownMs = now;
      longPressFired = false;
    } else {
      if (!longPressFired && (now - btnDownMs >= LONG_PRESS_MS)) {
        longPressFired = true;
        runFullCalibrationAndSave();
        renderPage(PAGE_SEQ[seqIndex]);
      }
    }
  } else {
    if (btnDownMs != 0) {
      unsigned long pressMs = now - btnDownMs;
      btnDownMs = 0;

      if (!longPressFired && pressMs < LONG_PRESS_MS) {
        seqIndex = (seqIndex + 1) % 4;
        renderPage(PAGE_SEQ[seqIndex]);
      }
    }
  }
}

// ======================================================
// Serial debug output
// ======================================================
static void serialDebugPrint(unsigned long nowMs) {
#if ENABLE_SERIAL_DEBUG
  static unsigned long lastMs = 0;
  if (nowMs - lastMs < 500) return;
  lastMs = nowMs;

  Serial.print("INA:");
  Serial.print(okIna ? "OK" : "NO");
  Serial.print(" ACC:");
  Serial.print(okAcc ? "OK" : "NO");
  Serial.print(" MAG:");
  Serial.print(okMag ? "OK" : "NO");

  Serial.print(" | Vbat:");
  Serial.print(vbat, 2);
  Serial.print("V ");
  Serial.print((int)lround(battPct));
  Serial.print("%");

  Serial.print(" | Isc:");
  Serial.print(iscmA_raw, 1);
  Serial.print("mA");

  Serial.print(" | Off:");
  Serial.print(iscOffsetValid ? iscOffset_mA : 0.0f, 1);
  Serial.print("mA");

  Serial.print(" | Irr:");
  Serial.print(irr_filt, 0);
  Serial.print("Wm2");

  Serial.print(" | Tilt:");
  Serial.print(tiltDeg_disp, 1);
  Serial.print("deg");

  Serial.print(" | Az:");
  Serial.print(azimDeg_disp, 1);
  Serial.print("deg");

  float heading = headingFromAzimuth(azimDeg_disp);
  Serial.print(" | Head:");
  Serial.print(heading, 1);
  Serial.print("deg");

  Serial.print(" | Cal(I T M):");
  Serial.print(iscOffsetValid ? "Y" : "N");
  Serial.print(" ");
  Serial.print(tiltCalValid ? "Y" : "N");
  Serial.print(" ");
  Serial.println(magCalValid ? "Y" : "N");
#endif
}

// ======================================================
// Arduino setup
// ======================================================
void setup() {
#if ENABLE_SERIAL_DEBUG
  Serial.begin(115200);
  delay(250);
  Serial.println();
  Serial.println("Solar Power Meter  Realtime + Calibration + Compass page");
#endif

  pinMode(PIN_BUTTON_GPIO, INPUT_PULLUP);

  pinMode(PIN_MOSFET_GPIO, OUTPUT);
  digitalWrite(PIN_MOSFET_GPIO, LOW);

  analogReadResolution(12);

  Wire.begin();
  Wire.setClock(400000);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
#if ENABLE_SERIAL_DEBUG
    Serial.println("OLED init failed");
#endif
    while (true) delay(100);
  }

  EEPROM.begin(EEPROM_BYTES);
  loadCalFromEEPROM();

  okIna = ina.begin();
  okAcc = accel.begin();
  okMag = mag.begin();

  showSplash();

  bool needFirstCal = (!iscOffsetValid) || (!tiltCalValid) || (!magCalValid);
  if (needFirstCal) {
    runFullCalibrationAndSave();
    loadCalFromEEPROM();
  }

  vbat = readBatteryVoltage();
  battPct = batteryPercentFromVoltage(vbat);

  updateTiltAndAzimuth();
  tiltDeg_disp = tiltDeg_raw;
  azimDeg_disp = azimDeg_raw;

  if (okIna) {
    iscmA_raw = measureIsc_mA_safe();
    float offset = iscOffsetValid ? iscOffset_mA : 0.0f;
    float iscCorr = iscmA_raw - offset;
    if (iscCorr < 0.0f) iscCorr = 0.0f;

    irr_now  = iscCorr * CAL_K_WM2_PER_MA;
    irr_filt = irr_now;
    irrMin = irr_filt;
    irrMax = irr_filt;
  } else {
    irr_now = irr_filt = 0.0f;
    irrMin = irrMax = NAN;
  }

  seqIndex = 0;
  renderPage(PAGE_SEQ[seqIndex]);

  unsigned long now = millis();
  lastUiMs = lastMeasMs = lastImuMs = lastBatMs = now;
}

// ======================================================
// Arduino main loop
// ======================================================
void loop() {
  unsigned long now = millis();

  handleButton();

  if (!inCalibration) {
    if (now - lastBatMs >= BAT_PERIOD_MS) {
      vbat = readBatteryVoltage();
      battPct = batteryPercentFromVoltage(vbat);
      lastBatMs = now;
    }

    if (now - lastImuMs >= IMU_PERIOD_MS) {
      updateTiltAndAzimuth();
      lastImuMs = now;
    }

    if (now - lastMeasMs >= MEAS_PERIOD_MS) {
      updateIrradiance();
      lastMeasMs = now;
    }

    if (now - lastUiMs >= UI_REFRESH_MS) {
      renderPage(PAGE_SEQ[seqIndex]);
      lastUiMs = now;
    }
  }

  serialDebugPrint(now);
  delay(1);
}
