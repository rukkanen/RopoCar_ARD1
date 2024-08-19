#include <Wire.h>
#include <I2Cdev.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <pt.h>
#include <BatteryManager.h>

// Motor Driver (TI SN754410NE) Pins
const int motor1PWM = 9;  // D9
const int motor1Dir = 8;  // D8
const int motor2PWM = 10; // D10
const int motor2Dir = 7;  // D7
const int motor3PWM = 6;  // D6
const int motor3Dir = 5;  // D5
const int motor4PWM = 4;  // D4
const int motor4Dir = 3;  // D3

// KY-033 IR Infrared Sensor Pins
const int sensor1Pin = A0; // A0 (Pin 19)
const int sensor2Pin = A1; // A1 (Pin 20)
const int sensor3Pin = A2; // A2 (Pin 21)
const int sensor4Pin = A3; // A3 (Pin 22)

// RCWL-0516 Microwave Radar Sensor Pin
const int radarPin = 11; // D11

// HC-SR04 Ultrasound Sensor Pins
const int ultrasoundTrig = 12; // D12
const int ultrasoundEcho = 13; // D13

// Servo for Ultrasound Rotation Pin
const int servoPin = 14; // D14
Servo swivelServo;

// Battery Voltage Monitoring Pin
const int batteryMonitorPin = A6; // A6 (Pin 24)

// I2C Communication Pins (SDA, SCL)
const int i2cSDA = 2; // D2
const int i2cSCL = 3; // D3

// SparkFun microSD Transflash Breakout Module Pins
const int sdCsPin = 15;   // D15
const int sdMosiPin = 16; // D16
const int sdMisoPin = 17; // D17
const int sdSckPin = 18;  // D18

// Drive Lights Pins
const int generalLightsPin = 19; // D19
const int drivingLightsPin = 20; // D20

// Global Variables and Flags
volatile bool motionDetected = false;
unsigned long lastMotionTime = 0;
const unsigned long radarTimeout = 5000;
pt ptMotorControl, ptRadar, ptUltrasound, ptLights, ptBatteryMonitor;
Adafruit_MPU6050 mpu;
BatteryManager batteryManager(batteryMonitorPin);
int motorBatteryThreshold = 20;

// Interrupt Service Routine for Radar
void radarISR()
{
  motionDetected = true;
  lastMotionTime = millis();
}

// Measure Distance with Ultrasound Sensor
long measureDistance()
{
  digitalWrite(ultrasoundTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(ultrasoundTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultrasoundTrig, LOW);

  long duration = pulseIn(ultrasoundEcho, HIGH);
  long distance = (duration * 0.034) / 2;
  return distance;
}

static int BatteryMonitorThread(struct pt *pt)
{
  PT_BEGIN(pt);

  while (1)
  {
    float batteryLevel = batteryManager.getBatteryAdjustedLevel();
    if (batteryLevel < motorBatteryThreshold)
    {
      // Logic to handle low battery, e.g., shut down motors or enter sleep mode
      Serial.println("Battery level critical, entering low power mode.");
    }
    PT_YIELD(pt); // Yield to allow other tasks to run
  }

  PT_END(pt);
}

// Protothread for Motor Control
static int MotorControlThread(struct pt *pt)
{
  PT_BEGIN(pt);

  while (1)
  {
    // Example motor control logic (to be expanded)
    if (motionDetected)
    {
      // Logic for moving the robot based on motor control
      digitalWrite(drivingLightsPin, HIGH); // Turn on driving lights
                                            // Add motor control logic here
    }
    PT_YIELD(pt);
  }

  PT_END(pt);
}

// Protothread for Radar Detection
static int RadarThread(struct pt *pt)
{
  PT_BEGIN(pt);

  while (1)
  {
    if (motionDetected)
    {
      Serial.println("Motion detected by radar!");
      // React to motion (e.g., wake up ARD2, start driving)
      motionDetected = false;
    }
    if (millis() - lastMotionTime > radarTimeout)
    {
      Serial.println("Radar not responding!");
    }
    PT_YIELD(pt);
  }

  PT_END(pt);
}

// Protothread for Ultrasound Mapping
static int UltrasoundThread(struct pt *pt)
{
  PT_BEGIN(pt);

  while (1)
  {
    float[20] sensorValues;
    // Swivel servo and take readings with the ultrasound sensor
    for (int angle = 0; angle <= 180; angle += 10)
    {
      swivelServo.write(angle);
      delay(500); // Allow servo to move
      long distance = measureDistance();
      Serial.print("Angle: ");
      Serial.print(angle);
      Serial.print(" - Distance: ");
      Serial.println(distance);
      PT_YIELD(pt);
    }
  }

  PT_END(pt);
}

// Protothread for Lights Control
static int LightsThread(struct pt *pt)
{
  PT_BEGIN(pt);

  while (1)
  {
    // Example logic for controlling LEDs
    digitalWrite(generalLightsPin, HIGH); // Turn on general lights
    delay(1000);
    digitalWrite(generalLightsPin, LOW); // Turn off general lights
    delay(1000);
    PT_YIELD(pt);
  }

  PT_END(pt);
}

void setup()
{
  Serial.begin(9600);
  Wire.begin(); // Initialize I2C

  attachInterrupt(digitalPinToInterrupt(radarPin), radarISR, RISING);

  // Initialize Protothreads
  PT_INIT(&ptMotorControl);
  PT_INIT(&ptRadar);
  PT_INIT(&ptUltrasound);
  PT_INIT(&ptLights);
  PT_INIT(&ptBatteryMonitor); // Initialize the Battery Monitor thread

  // Initialize Servo
  swivelServo.attach(servoPin);

  // Initialize MPU6050
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
  }
  Serial.println("MPU6050 Found!");

  // Initialize Lights
  pinMode(generalLightsPin, OUTPUT);
  pinMode(drivingLightsPin, OUTPUT);

  // Initialize Motors
  pinMode(motor1Dir, OUTPUT);
  pinMode(motor2Dir, OUTPUT);
  pinMode(motor3Dir, OUTPUT);
  pinMode(motor4Dir, OUTPUT);
  analogWrite(motor1PWM, 0);
  analogWrite(motor2PWM, 0);
  analogWrite(motor3PWM, 0);
  analogWrite(motor4PWM, 0);
}

void loop()
{
  PT_SCHEDULE(MotorControlThread(&ptMotorControl));
  PT_SCHEDULE(RadarThread(&ptRadar));
  PT_SCHEDULE(UltrasoundThread(&ptUltrasound));
  PT_SCHEDULE(LightsThread(&ptLights));
  PT_SCHEDULE(BatteryMonitorThread(&ptBatteryMonitor)); // Schedule Battery Monitor thread

  // Add more threads or logic as needed
}
