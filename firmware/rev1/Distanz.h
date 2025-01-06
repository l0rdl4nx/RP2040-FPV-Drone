#include "Arduino.h"
#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

class DISTANZ {
  long duration;
  int trigpin;
  int echopin;
  unsigned long vl53l0x_entfernung;

public:
  DISTANZ(int trig_k, int echo_k) {
    Wire.begin();
    trigpin = trig_k;
    echopin = echo_k;
  }

  void vl53l0x_init() {
    sensor.setTimeout(500);
    sensor.init();
    sensor.setMeasurementTimingBudget(200000);
  }

  float distanzmessung() {
    digitalWrite(trigpin, LOW);
    delayMicroseconds(5);
    digitalWrite(trigpin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigpin, LOW);
    duration = pulseIn(echopin, HIGH);
    return ((duration / 2) * 0.03432);
  }

  unsigned long get_vl53l0xdistanz() {
    return (sensor.readRangeSingleMillimeters());
  }
};
