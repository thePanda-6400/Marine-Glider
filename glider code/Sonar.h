
#ifndef Sonar_h
#define Sonar_h

#include "Arduino.h"
#include "ping1d.h"
#include "HardwareSerial.h"

class Sonar {
public:
    Sonar(HardwareSerial& serialPort); // Constructor that takes a reference to a Serial port
    bool initialize();
    int getDistance();

private:
    Ping1D ping;
};

#endif
