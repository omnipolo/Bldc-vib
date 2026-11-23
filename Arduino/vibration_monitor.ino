
#include <Wire.h>
#include <math.h>

#define MPU_ADDR 0x68

const int SAMPLES = 400;
const float ACCEL_SCALE = 16384.0;
const float G = 9.80665;

void writeRegister(byte reg, byte value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

int16_t readWord() {
  int16_t value = Wire.read() << 8;
  value |= Wire.read();
  return value;
}

bool readMPU(int16_t &ax, int16_t &ay, int16_t &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);

  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom(MPU_ADDR, 6) != 6) return false;

  ax = readWord();
  ay = readWord();
  az = readWord();

  return true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);

  delay(100);

  writeRegister(0x6B, 0x00);
  writeRegister(0x1C, 0x00);
  writeRegister(0x1A, 0x03);
  writeRegister(0x19, 0x04);

  delay(100);

  Serial.println("MPU6050 VIBRATION MONITOR");
  Serial.println("X,Y,Z,TOTAL (m/s2)");
}

void loop() {
  double sumX = 0, sumY = 0, sumZ = 0;
  double sumX2 = 0, sumY2 = 0, sumZ2 = 0;

  for (int i = 0; i < SAMPLES; i++) {
    int16_t ax, ay, az;

    if (!readMPU(ax, ay, az)) {
      Serial.println("MPU CONNECTION ERROR");
      delay(1000);
      return;
    }

    double x = (ax / ACCEL_SCALE) * G;
    double y = (ay / ACCEL_SCALE) * G;
    double z = (az / ACCEL_SCALE) * G;

    sumX += x;
    sumY += y;
    sumZ += z;

    sumX2 += x * x;
    sumY2 += y * y;
    sumZ2 += z * z;

    delayMicroseconds(5000);
  }

  double meanX = sumX / SAMPLES;
  double meanY = sumY / SAMPLES;
  double meanZ = sumZ / SAMPLES;

  double rmsX = sqrt(max(0.0, sumX2 / SAMPLES - meanX * meanX));
  double rmsY = sqrt(max(0.0, sumY2 / SAMPLES - meanY * meanY));
  double rmsZ = sqrt(max(0.0, sumZ2 / SAMPLES - meanZ * meanZ));

  double totalRMS = sqrt(
    rmsX * rmsX +
    rmsY * rmsY +
    rmsZ * rmsZ
  );

  Serial.print(rmsX, 4);
  Serial.print(",");
  Serial.print(rmsY, 4);
  Serial.print(",");
  Serial.print(rmsZ, 4);
  Serial.print(",");
  Serial.println(totalRMS, 4);
}
