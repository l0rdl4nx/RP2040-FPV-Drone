// Drohne
//  Mikrocontroller-Kurs 2019-24

#include <nRF24L01.h>
#include <RF24.h>
#include <PID_v1.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include "Distanz.h"
#include "Umweltmessung.h"
#include "comm_RX.h"
#include <Adafruit_BME280.h>

RF24 radio(7, 8);  // CE, CSN
MPU6050 mpu6050(Wire);
Adafruit_BME280 bme280;
DISTANZ distanz(4, 3);   // TRIG, ECHO
UMWELT umwelt = UMWELT(A0, 2200);  // MQ-7 Pin, Widerstand R2 in Ohm

////////////////////////////////////NeoPixel////////////////////////////////////
#define strip_pin1 1
#define strip_pin2 1

Adafruit_NeoPixel strip1(4, strip_pin1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2(25, strip_pin2, NEO_GRB + NEO_KHZ800);

////////////////////////////////////Batterie////////////////////////////////////
#define lipo_pin A1
float lipo_spannung = 0;
int lipo_value = 0;

/////////////////////////////////////Umwelt/////////////////////////////////////
float co_ppm;
float Temperatur = 0;
float Pascal = 0;
int Feuchtigkeit = 0;
float hoehe, aktuell_h, start_h;
float druck = 0;

/////////////////////////////////Distanzmessung/////////////////////////////////
int entfernung_1;
int entfernung_2;

/////////////////////////////////////Motor/////////////////////////////////////
#define L_V_prop 9
#define L_H_prop 10
#define R_V_prop 6
#define R_H_prop 5

int pwm_L_V;
int pwm_L_H;
int pwm_R_V;
int pwm_R_H;

/////////////////////////////////////PID/////////////////////////////////////
double rollSetpoint, rollInput, rollOutput;
double pitchSetpoint, pitchInput, pitchOutput;
double yawSetpoint, yawInput, yawOutput;
double autoSetpoint, autoInput, autoOutput;

double Kp = 0.5, Ki = 0.0005, Kd = 0.01;

PID rollPID(&rollInput, &rollOutput, &rollSetpoint, Kp, Ki, Kd, DIRECT);
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, Kp, Ki, Kd, DIRECT);
PID yawPID(&yawInput, &yawOutput, &yawSetpoint, Kp, Ki, Kd, DIRECT);
PID autoPID(&autoInput, &autoOutput, &autoSetpoint, Kp, Ki, Kd, DIRECT);

//Roll, Pitch, Throttle
float roll_desired_angle;
float pitch_desired_angle;
float yaw_desired_angle;
float thr_desired_angle;
int takeoff_height;
int jly = 600;
int jrx = 512;
int jry = 512;
//-----------------------------------RF24-------------------------------------
/////////////////////////////////////RF24/////////////////////////////////////
const byte ip[][6] = { "00001", "00002" };

struct datapack {
  byte jlx;
  byte jly;
  byte jlb;
  byte jrx;
  byte jry;
  byte jrb;
  byte b1;
  byte b2;
  byte b3;
  byte b4;
  byte b5;
  char to[16];
  char tu[16];
};

datapack data;

void setup() {
  radio.begin();
  Serial.begin(115000);
  Wire.begin();
  mpu6050.begin();
  analogWriteResolution(255);
  //bme280.init();
  distanz.vl53l0x_init();
  umwelt.intialize();

  strip1.begin();
  strip1.show();
  strip1.setBrightness(150);
  strip2.begin();
  strip2.show();
  strip2.setBrightness(50);

  radio.openWritingPipe(ip[0]);
  radio.openReadingPipe(1, ip[1]);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_250KBPS);
  radio.startListening();

  pinMode(L_V_prop, OUTPUT);
  pinMode(L_H_prop, OUTPUT);
  pinMode(R_V_prop, OUTPUT);
  pinMode(R_H_prop, OUTPUT);
  analogWrite(L_V_prop, 0);
  analogWrite(L_H_prop, 0);
  analogWrite(R_V_prop, 0);
  analogWrite(R_H_prop, 0);

  pitchInput = 0.0;
  rollInput = 0.0;
  pitchSetpoint = 0.0;
  rollSetpoint = 0.0;
  yawInput = 0.0;
  yawSetpoint = 0.0;

  pitchPID.SetMode(AUTOMATIC);
  rollPID.SetMode(AUTOMATIC);
  yawPID.SetMode(AUTOMATIC);
  autoPID.SetMode(AUTOMATIC);

  pitchPID.SetOutputLimits(-15, 15);
  rollPID.SetOutputLimits(-15, 15);
  yawPID.SetOutputLimits(-15, 15);

  mpu6050.calcGyroOffsets();

  druck = bme280.readPressure();
  start_h = bme280.readAltitude(druck);
}

void loop() {
  //NRF24L01_Read
  if (radio.available()) {
    radio.read(&data, sizeof(datapack));
  }

  //Umwelt und Höhe
  co_ppm = umwelt.get_ppm();

  //Distanzmessung
  entfernung_1 = distanz.distanzmessung();
  entfernung_2 = distanz.get_vl53l0xdistanz();

  //MPU6050_Read
  mpu6050.update();

  roll_desired_angle = map(jrx, 0, 1024, -10, 10);
  pitch_desired_angle = map(jry, 0, 1024, -10, 10);
  yaw_desired_angle = map(data.jlx, 0, 1024, -10, 10);
  thr_desired_angle = map(jly, 0, 1024, -255, 255);

  if (entfernung_1 <= 15 && pitch_desired_angle > 0) pitch_desired_angle = 0;

  if (thr_desired_angle <= 0) thr_desired_angle = 0;

  rollSetpoint = roll_desired_angle;
  pitchSetpoint = pitch_desired_angle;
  rollInput = mpu6050.getAngleX();
  pitchInput = mpu6050.getAngleY();

  rollPID.Compute();
  pitchPID.Compute();

  pwm_L_V = thr_desired_angle + (pitchOutput) + (rollOutput) + (yaw_desired_angle);
  pwm_L_H = thr_desired_angle - (pitchOutput) + (rollOutput) - (yaw_desired_angle);
  pwm_R_V = thr_desired_angle + (pitchOutput) - (rollOutput) - (yaw_desired_angle);
  pwm_R_H = thr_desired_angle - (pitchOutput) - (rollOutput) + (yaw_desired_angle);

  analogWrite(L_V_prop, pwm_L_V);
  analogWrite(L_H_prop, pwm_L_H);
  analogWrite(R_V_prop, pwm_R_V);
  analogWrite(R_H_prop, pwm_R_H);

  if (yaw_desired_angle != 0) yawSetpoint = mpu6050.getAngleZ();
  yawInput = mpu6050.getAngleZ();
  yawPID.Compute();

  pwm_L_V = pwm_L_V + (yawOutput);
  pwm_L_H = pwm_L_H - (yawOutput);
  pwm_R_V = pwm_R_V - (yawOutput);
  pwm_R_H = pwm_R_H + (yawOutput);

  analogWrite(L_V_prop, pwm_L_V);
  analogWrite(L_H_prop, pwm_L_H);
  analogWrite(R_V_prop, pwm_R_V);
  analogWrite(R_H_prop, pwm_R_H);

  Serial.print("LV: ");
  Serial.print(pwm_L_V);
  Serial.print(" | ");
  Serial.print("LH: ");
  Serial.print(pwm_L_H);
  Serial.print(" | ");
  Serial.print("RV: ");
  Serial.print(pwm_R_V);
  Serial.print(" | ");
  Serial.print("RH: ");
  Serial.print(pwm_R_H);
  Serial.print("   |   ");
  Serial.print("ROLL: ");
  Serial.print(rollOutput);
  Serial.print(" | ");
  Serial.print("PITCH: ");
  Serial.print(pitchOutput);
  Serial.print(" | ");
  Serial.print("THROTTLE: ");
  Serial.print(thr_desired_angle);
  Serial.print(" | ");
  Serial.print("Xº: ");
  Serial.print(rollInput);
  Serial.print(" | ");
  Serial.print("Yº: ");
  Serial.print(pitchInput);
  Serial.print(" | ");
  Serial.print("Zº: ");
  Serial.print(yawInput);
  Serial.println(" ");
}

void data_send() {
  radio.stopListening();
  radio.write(&data, sizeof(datapack));
  radio.startListening();
}

void licht() {
}
