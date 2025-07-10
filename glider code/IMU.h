
#ifndef IMU_h
#define IMU_h

#include "Arduino.h"
#include "Wire.h"

class IMU {
public:
    IMU(); // Constructor
    void begin(); // Initialize IMU settings
    void readAccelerometer(int &ax, int &ay, int &az);
    void readGyroscope(int &gx, int &gy, int &gz);
    void readMagnetometer(int &mx, int &my, int &mz);

private:
    void writeRegister(byte addr, byte reg, byte data);
    void readData(byte addr, byte reg, byte num, byte *buf);
};

#endif
