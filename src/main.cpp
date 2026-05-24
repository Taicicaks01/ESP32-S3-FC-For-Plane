#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMI270.h>
#include <TinyGPS++.h>

namespace {

constexpr uint32_t kTelemetryBaud = 115200;
constexpr uint32_t kElrsBaud = 420000;
constexpr uint32_t kGpsBaud = 115200;

constexpr int kElrsRxPin = 18;
constexpr int kElrsTxPin = 17;
constexpr int kGpsRxPin = 16;
constexpr int kGpsTxPin = 15;

constexpr int kThrottlePin = 6;
constexpr int kAileronPin = 7;
constexpr int kElevatorPin = 8;
constexpr int kRudderPin = 9;

constexpr int kServoFreq = 50;
constexpr int kServoResBits = 16;
constexpr int kThrottleChannel = 0;
constexpr int kAileronChannel = 1;
constexpr int kElevatorChannel = 2;
constexpr int kRudderChannel = 3;

constexpr uint8_t kBmeAddress = 0x76;
constexpr uint8_t kCompassAddress = 0x1E;  // HMC5883L
constexpr uint8_t kCrsfRcChannelsPacked = 0x16;

struct AttitudeData {
  float ax = 0.0f;
  float ay = 0.0f;
  float az = 0.0f;
  float gx = 0.0f;
  float gy = 0.0f;
  float gz = 0.0f;
};

struct EnvironmentData {
  float temperatureC = 0.0f;
  float pressurePa = 0.0f;
  float altitudeM = 0.0f;
};

struct CompassData {
  float headingDeg = 0.0f;
  float mx = 0.0f;
  float my = 0.0f;
  float mz = 0.0f;
};

struct GpsData {
  bool validFix = false;
  double latitude = 0.0;
  double longitude = 0.0;
  double altitudeM = 0.0;
  double speedMps = 0.0;
  uint8_t satellites = 0;
};

struct RcData {
  uint16_t ch[16] = {1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000,
                     1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
  uint32_t lastUpdateMs = 0;
  bool signalValid = false;
};

struct CommandState {
  bool armed = false;
  float rollTrim = 0.0f;
  float pitchTrim = 0.0f;
  float yawTrim = 0.0f;
};

Adafruit_BME280 bme;
Adafruit_BMI270 bmi;
TinyGPSPlus gps;
RcData rc;
CommandState cmd;

AttitudeData attitude;
EnvironmentData environment;
CompassData compass;
GpsData gpsData;

uint8_t crc8DvbS2(const uint8_t* data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; ++b) {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0xD5) : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

void decodeCrsfChannels(const uint8_t* payload, uint16_t* channels) {
  uint32_t bitBuffer = 0;
  uint8_t bitCount = 0;
  int ch = 0;

  for (int i = 0; i < 22 && ch < 16; ++i) {
    bitBuffer |= static_cast<uint32_t>(payload[i]) << bitCount;
    bitCount += 8;
    while (bitCount >= 11 && ch < 16) {
      channels[ch++] = static_cast<uint16_t>(bitBuffer & 0x7FF);
      bitBuffer >>= 11;
      bitCount -= 11;
    }
  }

  for (int i = 0; i < 16; ++i) {
    channels[i] = constrain(map(channels[i], 172, 1811, 988, 2012), 988, 2012);
  }
}

void parseElrsData() {
  static enum { WAIT_ADDR, WAIT_LEN, WAIT_DATA } state = WAIT_ADDR;
  static uint8_t address = 0;
  static uint8_t length = 0;
  static uint8_t frame[64];
  static uint8_t index = 0;

  while (Serial1.available() > 0) {
    uint8_t b = static_cast<uint8_t>(Serial1.read());
    switch (state) {
      case WAIT_ADDR:
        address = b;
        state = WAIT_LEN;
        break;
      case WAIT_LEN:
        length = b;
        index = 0;
        if (length < 2 || length > sizeof(frame)) {
          state = WAIT_ADDR;
          break;
        }
        state = WAIT_DATA;
        break;
      case WAIT_DATA:
        frame[index++] = b;
        if (index == length) {
          uint8_t type = frame[0];
          uint8_t expectedCrc = crc8DvbS2(frame, length - 1);
          uint8_t receivedCrc = frame[length - 1];
          if (expectedCrc == receivedCrc && type == kCrsfRcChannelsPacked) {
            decodeCrsfChannels(&frame[1], rc.ch);
            rc.lastUpdateMs = millis();
            rc.signalValid = true;
          }
          (void)address;
          state = WAIT_ADDR;
        }
        break;
    }
  }

  if (millis() - rc.lastUpdateMs > 500) {
    rc.signalValid = false;
  }
}

void parseGpsData() {
  while (Serial2.available() > 0) {
    gps.encode(static_cast<char>(Serial2.read()));
  }

  gpsData.validFix = gps.location.isValid() && gps.altitude.isValid();
  if (gpsData.validFix) {
    gpsData.latitude = gps.location.lat();
    gpsData.longitude = gps.location.lng();
    gpsData.altitudeM = gps.altitude.meters();
  }
  gpsData.speedMps = gps.speed.mps();
  gpsData.satellites = gps.satellites.value();
}

bool initCompass() {
  Wire.beginTransmission(kCompassAddress);
  if (Wire.endTransmission() != 0) {
    return false;
  }

  Wire.beginTransmission(kCompassAddress);
  Wire.write(0x00);  // CRA
  Wire.write(0x70);  // 8 samples @ 15Hz
  Wire.endTransmission();

  Wire.beginTransmission(kCompassAddress);
  Wire.write(0x01);  // CRB
  Wire.write(0x20);  // Gain
  Wire.endTransmission();

  Wire.beginTransmission(kCompassAddress);
  Wire.write(0x02);  // Mode
  Wire.write(0x00);  // Continuous mode
  Wire.endTransmission();

  return true;
}

bool readCompass() {
  Wire.beginTransmission(kCompassAddress);
  Wire.write(0x03);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(kCompassAddress, static_cast<uint8_t>(6)) != 6) {
    return false;
  }

  int16_t x = (Wire.read() << 8) | Wire.read();
  int16_t z = (Wire.read() << 8) | Wire.read();
  int16_t y = (Wire.read() << 8) | Wire.read();

  compass.mx = static_cast<float>(x);
  compass.my = static_cast<float>(y);
  compass.mz = static_cast<float>(z);
  compass.headingDeg = atan2f(compass.my, compass.mx) * 180.0f / PI;
  if (compass.headingDeg < 0.0f) {
    compass.headingDeg += 360.0f;
  }
  return true;
}

void writeServoMicros(int channel, int micros) {
  micros = constrain(micros, 1000, 2000);
  const uint32_t periodMicros = 1000000UL / kServoFreq;
  const uint32_t maxDuty = (1UL << kServoResBits) - 1UL;
  const uint32_t duty = (static_cast<uint32_t>(micros) * maxDuty) / periodMicros;
  ledcWrite(channel, duty);
}

void mixerAndOutput() {
  if (!cmd.armed || !rc.signalValid) {
    writeServoMicros(kThrottleChannel, 1000);
    writeServoMicros(kAileronChannel, 1500);
    writeServoMicros(kElevatorChannel, 1500);
    writeServoMicros(kRudderChannel, 1500);
    return;
  }

  const float rollCmd = (static_cast<int>(rc.ch[0]) - 1500) + cmd.rollTrim;
  const float pitchCmd = (static_cast<int>(rc.ch[1]) - 1500) + cmd.pitchTrim;
  const float yawCmd = (static_cast<int>(rc.ch[3]) - 1500) + cmd.yawTrim;
  const int throttleCmd = rc.ch[2];

  const float gyroStabRoll = -0.08f * attitude.gx;
  const float gyroStabPitch = -0.08f * attitude.gy;
  const float gyroStabYaw = -0.05f * attitude.gz;

  writeServoMicros(kThrottleChannel, throttleCmd);
  writeServoMicros(kAileronChannel, 1500 + static_cast<int>(rollCmd + gyroStabRoll));
  writeServoMicros(kElevatorChannel, 1500 + static_cast<int>(pitchCmd + gyroStabPitch));
  writeServoMicros(kRudderChannel, 1500 + static_cast<int>(yawCmd + gyroStabYaw));
}

void printTelemetry() {
  static uint32_t lastTx = 0;
  if (millis() - lastTx < 100) {
    return;
  }
  lastTx = millis();

  Serial.print("FC,");
  Serial.print(millis());
  Serial.print(',');
  Serial.print(cmd.armed ? 1 : 0);
  Serial.print(',');
  Serial.print(rc.signalValid ? 1 : 0);
  Serial.print(',');
  Serial.print(attitude.ax, 3);
  Serial.print(',');
  Serial.print(attitude.ay, 3);
  Serial.print(',');
  Serial.print(attitude.az, 3);
  Serial.print(',');
  Serial.print(attitude.gx, 3);
  Serial.print(',');
  Serial.print(attitude.gy, 3);
  Serial.print(',');
  Serial.print(attitude.gz, 3);
  Serial.print(',');
  Serial.print(environment.temperatureC, 2);
  Serial.print(',');
  Serial.print(environment.pressurePa, 1);
  Serial.print(',');
  Serial.print(environment.altitudeM, 2);
  Serial.print(',');
  Serial.print(compass.headingDeg, 1);
  Serial.print(',');
  Serial.print(gpsData.validFix ? gpsData.latitude : 0.0, 6);
  Serial.print(',');
  Serial.print(gpsData.validFix ? gpsData.longitude : 0.0, 6);
  Serial.print(',');
  Serial.print(gpsData.altitudeM, 2);
  Serial.print(',');
  Serial.print(gpsData.speedMps, 2);
  Serial.print(',');
  Serial.print(gpsData.satellites);
  Serial.print(',');
  Serial.print(rc.ch[0]);
  Serial.print(',');
  Serial.print(rc.ch[1]);
  Serial.print(',');
  Serial.print(rc.ch[2]);
  Serial.print(',');
  Serial.println(rc.ch[3]);
}

void processGroundCommands() {
  static String line;
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (line.length() == 0) {
        continue;
      }

      line.trim();
      if (line == "ARM 1") {
        cmd.armed = true;
      } else if (line == "ARM 0") {
        cmd.armed = false;
      } else if (line.startsWith("TRIM ")) {
        float r, p, y;
        if (sscanf(line.c_str(), "TRIM %f %f %f", &r, &p, &y) == 3) {
          cmd.rollTrim = r;
          cmd.pitchTrim = p;
          cmd.yawTrim = y;
        }
      }

      Serial.print("ACK,");
      Serial.println(line);
      line = "";
    } else {
      line += c;
    }
  }
}

void updateSensors() {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  if (bmi.getEvent(&accel, &gyro, &temp)) {
    attitude.ax = accel.acceleration.x;
    attitude.ay = accel.acceleration.y;
    attitude.az = accel.acceleration.z;
    attitude.gx = gyro.gyro.x * RAD_TO_DEG;
    attitude.gy = gyro.gyro.y * RAD_TO_DEG;
    attitude.gz = gyro.gyro.z * RAD_TO_DEG;
  }

  environment.temperatureC = bme.readTemperature();
  environment.pressurePa = bme.readPressure();
  environment.altitudeM = bme.readAltitude(1013.25f);

  readCompass();
}

}  // namespace

void setup() {
  Serial.begin(kTelemetryBaud);
  Serial1.begin(kElrsBaud, SERIAL_8N1, kElrsRxPin, kElrsTxPin);
  Serial2.begin(kGpsBaud, SERIAL_8N1, kGpsRxPin, kGpsTxPin);

  Wire.begin();

  const bool bmeOk = bme.begin(kBmeAddress);
  const bool bmiOk = bmi.begin_I2C();
  const bool compassOk = initCompass();

  ledcSetup(kThrottleChannel, kServoFreq, kServoResBits);
  ledcSetup(kAileronChannel, kServoFreq, kServoResBits);
  ledcSetup(kElevatorChannel, kServoFreq, kServoResBits);
  ledcSetup(kRudderChannel, kServoFreq, kServoResBits);

  ledcAttachPin(kThrottlePin, kThrottleChannel);
  ledcAttachPin(kAileronPin, kAileronChannel);
  ledcAttachPin(kElevatorPin, kElevatorChannel);
  ledcAttachPin(kRudderPin, kRudderChannel);

  writeServoMicros(kThrottleChannel, 1000);
  writeServoMicros(kAileronChannel, 1500);
  writeServoMicros(kElevatorChannel, 1500);
  writeServoMicros(kRudderChannel, 1500);

  Serial.println("ESP32-S3 Plane FC boot");
  Serial.printf("Sensors: BMI=%d BME280=%d Compass=%d\n", bmiOk ? 1 : 0, bmeOk ? 1 : 0, compassOk ? 1 : 0);
}

void loop() {
  parseElrsData();
  parseGpsData();
  processGroundCommands();

  static uint32_t lastControlMs = 0;
  if (millis() - lastControlMs >= 4) {  // 250Hz
    lastControlMs = millis();
    updateSensors();
    mixerAndOutput();
  }

  printTelemetry();
}
