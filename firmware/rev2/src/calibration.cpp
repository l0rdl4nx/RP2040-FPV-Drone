#include <Arduino.h>
#include <MPU9250.h>
#include <LittleFS.h>
#include <stabilization.h>

float angles[3];
float pid[4];

config conf;
stabilization stab(&angles, &pid, &conf);

boolean debug = true;

void print_calibration()
{
  Serial.println("< calibration parameters >");
  Serial.println("accel bias [g]: ");
  Serial.print(conf.acc[X] * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.print(", ");
  Serial.print(conf.acc[Y] * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.print(", ");
  Serial.print(conf.acc[Z] * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.println();
  Serial.println("gyro bias [deg/s]: ");
  Serial.print(conf.gyro[X] / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.print(", ");
  Serial.print(conf.gyro[Y] / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.print(", ");
  Serial.print(conf.gyro[Z] / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.println();
  Serial.println("mag bias [mG]: ");
  Serial.print(conf.mag[X]);
  Serial.print(", ");
  Serial.print(conf.mag[Y]);
  Serial.print(", ");
  Serial.print(conf.mag[Z]);
  Serial.println();
  Serial.println("mag scale []: ");
  Serial.print(conf.magScale[X]);
  Serial.print(", ");
  Serial.print(conf.magScale[Y]);
  Serial.print(", ");
  Serial.print(conf.magScale[Z]);
  Serial.println();
}

void setup()
{  
  Serial.begin(115200);
  Wire.begin();

  delay(5000);

  stab.setDebug(debug);
  stab.formatFS();

  if (!stab.Start())
    while (1)
      ;

  delay(2000);

  stab.calibrateMPU();
  print_calibration();
}

void loop()
{
}
