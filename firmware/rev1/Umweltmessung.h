#include "Arduino.h"

class UMWELT {
  int sensorValue = 0;
  float sensor_volt = 0;
  float RS_gas = 0;
  float R0 = 0;
  int R2 = 0;
  int mq7_pin = 0;
  float ratio = 0;
  float x = 0;

public:
  UMWELT(int r2, int mq7) {
    R2 = r2;
    mq7_pin = mq7;
  }

  void intialize() {
    sensorValue = analogRead(mq7_pin);
    sensor_volt = (float)sensorValue / 1024 * 5.0;
    RS_gas = ((5.0 * R2) / sensor_volt) - R2;
    R0 = RS_gas / 1;
  }

  float get_ppm() {
    sensorValue = analogRead(mq7_pin);
    sensor_volt = sensorValue / 1024 * 5.0;
    RS_gas = (5.0 - sensor_volt) / sensor_volt;
    ratio = RS_gas / R0;
    x = 1538.46 * ratio;
    return (pow(x, -1.709));
  }
};
