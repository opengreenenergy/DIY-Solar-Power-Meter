# DIY Solar Power Meter

A compact handheld solar irradiance meter built using the **Seeed XIAO ESP32-C3**, **INA226**, **LSM303AGR**, and either a **Waveshare 1.54 inch 240x240 ST7789 TFT LCD** or a **128x64 OLED display**.

This project is designed for measuring solar irradiance in the field and also displaying **tilt**, **azimuth**, and **battery level** in a compact handheld device.

---

## Open Green Energy

This project is developed by **Open Green Energy**.

## Full Project Documentation

Step by step build guide is available here:

https://www.instructables.com/DIY-Solar-Power-Meter/

---

## 3D Printed Enclosure

Download the 3D printable enclosure files here:

https://www.printables.com/model/1603265-diy-solar-power-meter

---

🌐 Website  
https://opengreenenergy.com

▶ YouTube Channel  
https://www.youtube.com/c/opengreenenergy

📷 Instagram  
https://instagram.com/opengreenenergy

❤️ Support Open Green Energy  
If you like this project and want to support development of more open hardware tools:

https://www.patreon.com/opengreenenergy

---

## Project Variants

This repository contains **two firmware versions** of the same project.

### 1. LCD Version
This version uses a **Waveshare 1.54 inch 240x240 ST7789 TFT LCD** and offers a more graphical user interface.

**Location:**  
`firmware/LCD_Version/`

**Main features:**
- 4 page UI
- Professional graphical display
- Boot screen with logo
- Weather style irradiance icon
- Home, Min/Max, Tilt/Azimuth/Isc, and Compass pages
- EEPROM based calibration

### 2. OLED Version
This version uses a **128x64 OLED display** and provides a simpler, low power interface.

**Location:**  
`firmware/OLED_Version/`

**Main features:**
- Compact monochrome UI
- 4 pages
- Battery display
- Irradiance, tilt, azimuth, and compass page
- Full calibration with EEPROM storage
- Display suitable for minimal hardware builds

---

## Features

- Measures solar irradiance in **W/m2**
- Measures **tilt angle** using accelerometer
- Measures **azimuth / heading** using magnetometer
- Battery voltage and battery percentage display
- Single button navigation
- Long press calibration
- EEPROM based calibration storage
- Two firmware variants for different displays
- Portable field measurement tool

---

## Hardware Used

- **MCU:** Seeed XIAO ESP32-C3
- **Current Sensor:** INA226
- **Accelerometer / Magnetometer:** LSM303AGR
- **Display Option 1:** Waveshare 1.54 inch LCD, 240x240, ST7789 SPI
- **Display Option 2:** 128x64 OLED display

---

## Working Principle

The meter estimates solar irradiance by measuring the **short circuit current** of a small reference solar panel through a shunt based current sensing arrangement using **INA226**.

The measured Isc value is corrected using a stored dark offset and then converted into irradiance using a calibration factor:

`Irradiance = Corrected Isc × Calibration Constant`

In the LCD firmware:

- **Calibration constant** = `15.05 W/m2 per mA`

In the OLED firmware, the calibration constant can be adjusted in code based on the reference panel used.

Tilt and azimuth are measured using accelerometer and magnetometer data, then filtered for stable display.

---

## Pin Connections

| Function | XIAO ESP32-C3 Pin |
|---------|-------------------|
| Battery Sense | D0 |
| MOSFET Gate | D1 |
| Push Button | D2 |

### LCD Version Additional Pins

| Function | XIAO ESP32-C3 Pin |
|---------|-------------------|
| LCD Reset | D3 |
| LCD CS | D6 |
| LCD DC | D7 |
| LCD SCK | D8 |
| LCD Backlight | D9 |
| LCD MOSI | D10 |

### OLED / Sensor I2C Bus

| Function | Connection |
|---------|------------|
| SDA | I2C SDA |
| SCL | I2C SCL |

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

## Battery Divider

Battery voltage is measured through a resistor divider:

- **Top resistor** = 200k
- **Bottom resistor** = 100k

Connection:

`VBAT -> 200k -> ADC pin D0 -> 100k -> GND`

---

## Button Operation

### LCD Version
- **Short Press** → Next page
- **Long Press for 2 seconds** → Calibration wizard

### OLED Version
- **Short Press** → Next page
- **Long Press for 2 seconds** → Full calibration

---

## UI Pages

### LCD Version Pages

#### 1. Home Page
Shows:
- Irradiance
- Tilt
- Azimuth
- Battery level

#### 2. Min / Max Page
Shows:
- Live irradiance
- Minimum irradiance
- Maximum irradiance

#### 3. Tilt / Azimuth / Isc Page
Shows:
- Tilt angle
- Azimuth angle
- Isc current

#### 4. Orientation Page
Shows:
- Compass style direction display
- Azimuth in degrees
- N / E / S / W direction

### OLED Version Pages

#### 1. Home Page
Shows:
- Battery percentage
- Irradiance
- Tilt

#### 2. Min / Max Page
Shows:
- Live irradiance
- Minimum irradiance
- Maximum irradiance

#### 3. Tilt / Azimuth Page
Shows:
- Tilt angle
- Azimuth angle

#### 4. Compass Page
Shows:
- Compass circle
- Needle direction
- Azimuth value

---

## Calibration Steps

Both firmware versions support calibration and store values in EEPROM.

### Step 1: Isc Offset
Cover the reference panel and save the dark current offset.

### Step 2: Tilt Zero
Keep the device level and save the zero tilt position.

### Step 3: Tilt Reference
Keep the device vertical and save the tilt reference position.

### Step 4: Compass Calibration
Rotate the device slowly so the magnetometer offset can be calculated.

---

## Libraries Required

Install these libraries from Arduino Library Manager:

- `Adafruit GFX Library`
- `Adafruit SSD1306`
- `Adafruit ST7735 and ST7789 Library`
- `Adafruit LIS2MDL`
- `Adafruit LSM303 Accelerometer`
- `Adafruit Unified Sensor`
- `EEPROM`
- `Wire`
- `SPI`
- `INA226.h`

---

## Firmware Structure

```text
DIY-Solar-Power-Meter
│
├── firmware
│   ├── LCD_Version
│   │   ├── solar_meter_lcd_v1.1.ino
│   │   └── oge_logo.h
│   │
│   ├── OLED_Version
│   │   └── solar_meter_oled_v1.0.ino
│   │
│   └── README.md
│
├── images
│
├── README.md
│
└── LICENSE
