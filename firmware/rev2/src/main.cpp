
/*------------------------------------ First Core --------------------------------------*/

#include <Arduino.h>
#include <CrsfSerial.h>

#define LED_BUILTIN 25 // for my custom RP2040 board

#define M_FL 6 // Front-Left
#define M_FR 7 // Front-Right
#define M_BL 8 // Back-Left
#define M_BR 9 // Back-Right

#define MAX_ANGLE 10

// create object for connecting to the receiver
// ELRS module connected on Serial1 (Pins 0 and 1, not important for USB connection other than most Arduino's)
// library-defined baudrate CRSF_BAUDRATE = 420000, should be changed if any other baudrate is defined for the receiver
CrsfSerial crsf(Serial1, CRSF_BAUDRATE);

boolean debug = true;

int rxThrottle, rxPitch, rxRoll, rxYaw;

float angles[3];
float pid[4];

// writes the received controller values to variables
void packetChannels()
{
  rxThrottle = crsf.getChannel(3);
  rxPitch = crsf.getChannel(4);
  rxRoll = crsf.getChannel(1);
  rxYaw = crsf.getChannel(2);
}

void setup()
{
  Serial.begin(115200);
  delay(4000);
  Serial.println("Start setup() of core 1.");
  crsf.begin();
  crsf.onPacketChannels = &packetChannels;
  Serial.println("Finish 1");
}

void loop()
{
  crsf.loop();
}

/*----------------------------------- Second Core --------------------------------------*/

#include <MPU9250.h>
#include <Servo.h>
#include <stabilization.h>

// create the struct object for storing every used configuration variable
config conf;

// create Servo and Stabilization objects
stabilization gyro(&angles, &pid, &conf);

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

float SetPoints[4] = {};

void initialize()
{
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);

  // attach the ESC connection using a Servo-class-member
  motor1.attach(M_FL, 1000, 2000);
  motor2.attach(M_FR, 1000, 2000);
  motor3.attach(M_BL, 1000, 2000);
  motor4.attach(M_BR, 1000, 2000);

  if (debug)
    Serial.println("Servos attached.");

  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);

  // start Stabilization, which calibrates gyro values and configures internal PID controls
  if (!gyro.Start())
  {
    if (debug)
      Serial.println("Failed configuring gyro, exiting program.");
    while (true)
      ;
  }

  if (debug)
    Serial.println("Calibration done.");

  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
}

void printGyro()
{
  gyro.getMPUAngles();

  Serial.print(angles[PITCH]);
  Serial.print("  |  ");
  Serial.print(angles[ROLL]);
  Serial.print("  |  ");
  Serial.println(angles[YAW]);
}

void printPID()
{
  Serial.print(pid[FRONT_LEFT]);
  Serial.print("  |  ");
  Serial.print(pid[FRONT_RIGHT]);
  Serial.print("  |  ");
  Serial.print(pid[BACK_LEFT]);
  Serial.print("  |  ");
  Serial.println(pid[BACK_RIGHT]);
}

void setup1()
{
  Serial.begin(115200);
  Wire.begin();

  delay(5000);

  if (debug)
    Serial.println("Start setup() of core 2.");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  initialize();

  SetPoints[PITCH] = 0;
  SetPoints[ROLL] = 0;
  SetPoints[YAW] = angles[YAW];
  SetPoints[THR] = 0;
  gyro.setSetpoints(SetPoints);

  if (debug)
    Serial.println("Finish 2");
}

void loop1()
{
  // gyro.Compute();
  // printGyro();
  // printPID();
  // delay(10);
  if (debug)
    Serial.println(conf.acc[X]);
  delay(5000);
}