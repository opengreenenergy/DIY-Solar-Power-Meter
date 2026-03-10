# OLED Firmware

This folder contains the firmware for the OLED display version of the DIY Solar Power Meter.

Main file:
solar_meter_oled_v1.0.ino

Display:
128x64 OLED (SSD1306)

Sensors:
INA226
LIS2MDL
LSM303 Accelerometer

# OLED Version Firmware

This firmware runs the **OLED display version** of the DIY Solar Power Meter.

Display used:
128x64 OLED (SSD1306)

## Features

- Solar irradiance measurement using INA226
- Tilt measurement using accelerometer
- Azimuth / heading using magnetometer
- Battery percentage display
- Compass page
- EEPROM based calibration
- Single button user interface

## Button Operation

Short Press  
Next page

Long Press (2 seconds)  
Start calibration procedure

## Calibration

The calibration procedure includes:

1. Irradiance dark offset
2. Tilt zero reference
3. Tilt reference (90°)
4. Magnetometer rotation calibration
