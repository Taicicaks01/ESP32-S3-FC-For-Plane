# ESP32-S3-FC-For-Plane

ESP32-S3 RC plane flight-controller scaffold with:
- BMI IMU (I2C)
- BME280 barometer (I2C)
- HMC5883L compass (I2C)
- Matek M10 GPS (UART)
- ELRS receiver via CRSF (UART)
- Ground-computer RX/TX telemetry + command link (USB serial)

## Firmware layout
- `platformio.ini` -> ESP32-S3 build target and required libraries
- `src/main.cpp` -> sensor IO, ELRS parsing, mixer, PWM outputs, telemetry protocol

## Pin defaults (change in `src/main.cpp`)
- ELRS UART: RX=18, TX=17
- GPS UART: RX=16, TX=15
- Servo outputs: THR=6, AIL=7, ELE=8, RUD=9

## Telemetry protocol (USB Serial @ 115200)
The controller sends CSV lines every 100 ms:

`FC,<ms>,<armed>,<rc_ok>,<imu...>,<env...>,<heading>,<gps...>,<rc channels...>`

Supported incoming commands:
- `ARM 1`
- `ARM 0`
- `TRIM <roll> <pitch> <yaw>`

## Build (PlatformIO)
```bash
pio run
pio device monitor -b 115200
```

## Notes
- CRSF RC channels are decoded from ELRS frames and used for control mixing.
- A basic gyro stabilization term is included as a starting point.
- Tune gains, calibrations, failsafe strategy, and airframe-specific mixing before flight.
