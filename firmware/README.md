# DIY Solar Power Meter

A compact handheld solar irradiance meter built using the **Seeed XIAO ESP32-C3**, **INA226**, **LIS2MDL + LSM303**, and a **Waveshare 1.54 inch 240x240 ST7789 TFT display**.

This project is designed for measuring solar irradiance in the field and also display **tilt**, **azimuth**, and **battery level** in a LCD interface.

---

## Full Project Documentation

Step by step build guide is available here:

https://www.instructables.com/DIY-Solar-Power-Meter/

---

## 3D Printed Enclosure

Download the 3D printable enclosure files here:

https://www.printables.com/model/1603265-diy-solar-power-meter

---

## Open Green Energy 

🌐 Website  
https://opengreenenergy.com

▶ YouTube Channel  
https://www.youtube.com/c/opengreenenergy

📷 Instagram  
https://instagram.com/opengreenenergy

---

## Features

- Measures solar irradiance in **W/m2**
- Measures **tilt angle** using accelerometer
- Measures **azimuth / heading** using magnetometer
- 4 page professional TFT display UI
- Single button navigation
- Long press calibration wizard
- EEPROM based calibration storage
  
---

## Hardware Used

- **MCU:** Seeed XIAO ESP32-C3
- **Display:** Waveshare 1.54 inch LCD, 240x240, ST7789 SPI
- **Current Sensor:** INA226
- **Accelerometer / Magnetometer:** Adafruit LSM303AGR
  
---

## Pin Connections

| Function | XIAO ESP32-C3 Pin |
|---------|-------------------|
| Battery Sense | D0 |
| MOSFET Gate | D1 |
| Push Button | D2 |
| LCD Reset | D3 |
| LCD CS | D6 |
| LCD DC | D7 |
| LCD SCK | D8 |
| LCD Backlight | D9 |
| LCD MOSI | D10 |

---

## LCD Wiring

| LCD Pin | XIAO ESP32-C3 |
|--------|----------------|
| VCC | 3V3 |
| GND | GND |
| DIN | D10 |
| CLK | D8 |
| CS | D6 |
| DC | D7 |
| RST | D3 |
| BL | D9 |

---

## INA226 Wiring

| INA226 Pin | Connection |
|-----------|------------|
| VCC | 3V3 |
| GND | GND |
| SDA | SDA |
| SCL | SCL |
| ADDR | 0x44 |

---

## Button Operation

- **Short Press** → Next page
- **Long Press for 2 seconds** → Calibration wizard

---

## UI Pages

### 1. Home Page
Shows:
- Irradiance
- Tilt
- Azimuth
- Battery level

### 2. Min / Max Page
Shows:
- Live irradiance
- Minimum irradiance
- Maximum irradiance

### 3. Tilt / Azimuth / Isc Page
Shows:
- Tilt angle
- Azimuth angle
- Isc current

### 4. Orientation Page
Shows:
- Compass style direction display
- Azimuth in degrees
- N / E / S / W direction

---

## Calibration Steps

The firmware includes a 4 step calibration wizard:

### Step 1: Isc Offset
Cover the reference panel and save the dark current offset.

### Step 2: Tilt Zero
Keep the device level and save the zero tilt position.

### Step 3: Tilt 90 Degree
Keep the device vertical and save the 90 degree tilt reference.

### Step 4: Compass Calibration
Rotate the device slowly so the magnetometer offset can be calculated.

All calibration values are stored in EEPROM.

---

## Libraries Required

Install these libraries from Arduino Library Manager:

- `Adafruit GFX Library`
- `Adafruit ST7735 and ST7789 Library`
- `Adafruit LIS2MDL`
- `Adafruit LSM303 Accelerometer`
- `Adafruit Unified Sensor`
- `EEPROM`
- `Wire`
- `SPI`
- `INA226.h`
---

## Firmware Settings

Inside the code you can change:

```cpp
#define ENABLE_SERIAL_DEBUG 1
#define DEMO_MODE 1
#define HEADER_WEATHER_ICON 0
