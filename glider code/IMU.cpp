
#include "IMU.h"

IMU::IMU() {}

void IMU::begin() {
    Wire.begin(); // Start I2C
    Serial.begin(9600); // Start serial for output

    // Initialize Accelerometer
    writeRegister(0x18, 0x0F, 0x03); // Set range
    writeRegister(0x18, 0x10, 0x08); // Set bandwidth

    // Initialize Gyroscope
    writeRegister(0x68, 0x0F, 0x04); // Set range
    writeRegister(0x68, 0x10, 0x07); // Set bandwidth

    // Initialize Magnetometer
    writeRegister(0x10, 0x4B, 0x83); // Soft reset
    delay(50); // Wait for reset to complete
    writeRegister(0x10, 0x4C, 0x00); // Normal mode
}

void IMU::writeRegister(byte addr, byte reg, byte data) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

void IMU::readData(byte addr, byte reg, byte num, byte *buf) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(addr, num);
    int i = 0;
    while (Wire.available()) {
        buf[i++] = Wire.read();
    }
}

void IMU::readAccelerometer(int &ax, int &ay, int &az) {
    byte buf[6];
    readData(0x18, 0x02, 6, buf);
    ax = (buf[1] << 8) | buf[0];
    ay = (buf[3] << 8) | buf[2];
    az = (buf[5] << 8) | buf[4];
}

void IMU::readGyroscope(int &gx, int &gy, int &gz) {
    byte buf[6];
    readData(0x68, 0x02, 6, buf);
    gx = (buf[1] << 8) | buf[0];
    gy = (buf[3] << 8) | buf[2];
    gz = (buf[5] << 8) | buf[4];
}

void IMU::readMagnetometer(int &mx, int &my, int &mz) {
    byte buf[6];
    readData(0x10, 0x42, 6, buf);
    mx = (buf[1] << 8) | buf[0];
    my = (buf[3] << 8) | buf[2];
    mz = (buf[5] << 8) | buf[4];
}
