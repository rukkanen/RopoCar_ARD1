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
// const int sensor4Pin = A3; // A3 (Pin 22)

// RCWL-0516 Microwave Radar Sensor Pin
const int radarPin = 21; // D11

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
int motorBatteryThreshold = 20;

pt ptMotorControl, ptSetup, ptQuick, ptUltrasound, ptLights, ptBatteryMonitor;

Adafruit_MPU6050 mpu;
BatteryManager batteryManager(batteryMonitorPin);

// Interrupt Service Routine for Radar
void radarISR()
{
  Serial.println("Radar Interrupt");
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

PT_THREAD(BatteryMonitorThread(struct pt *pt))
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
PT_THREAD(MotorControlThread(struct pt *pt))
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

// Protothread for Ultrasound Mapping
PT_THREAD(UltrasoundThread(struct pt *pt))
{
  PT_BEGIN(pt);

  while (1)
  {
    // float[20] sensorValues;
    // int sensorIndex;
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
PT_THREAD(LightsThread(struct pt *pt))
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

// Protothread for Radar Detection
PT_THREAD(Quickhread(struct pt *pt))
{
  static unsigned long startTime = millis(); // Use unsigned long for millis() to avoid overflow
  PT_BEGIN(pt);

  while (1)
  {

    // if (digitalRead(radarPin) == HIGH)
    if (motionDetected)
    {
      Serial.println("Motion detected!");
      motionDetected = false;
    }
    else
    {
      Serial.println("No motion detected.");
    }
    PT_YIELD(pt);
    // Get data from MPU6050
    sensors_event_t accelEvent, gyroEvent;
    PT_YIELD(pt);
    sensors_event_t temp;
    PT_YIELD(pt);
    mpu.getEvent(&accelEvent, &gyroEvent, &temp);
    PT_YIELD(pt);
    // Print accelerometer data
    Serial.print("Accelerometer (m/s^2): ");
    Serial.print(accelEvent.acceleration.x);
    Serial.print(", ");
    Serial.print(accelEvent.acceleration.y);
    Serial.print(", ");
    Serial.println(accelEvent.acceleration.z);
    PT_YIELD(pt);
    // Print gyroscope data
    Serial.print("Gyroscope (rad/s): ");
    Serial.print(gyroEvent.gyro.x);
    Serial.print(", ");
    Serial.print(gyroEvent.gyro.y);
    Serial.print(", ");
    Serial.println(gyroEvent.gyro.z);

    // PT_YIELD(pt, millis() - startTime > 2000);
    PT_YIELD(pt);
    startTime = millis(); // Update the start time for the next iteration
  }

  PT_END(pt);
}

PT_THREAD(SetupThread(struct pt *pt))
{
  PT_BEGIN(pt);
  Serial.begin(9600);
  PT_YIELD(pt);

  Wire.begin(); // Initialize I2C
  Serial.println("Initializing...");
  PT_YIELD(pt);

  pinMode(radarPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(radarPin), radarISR, RISING);
  PT_YIELD(pt);
  PT_END(pt);
}

void setup()
{

  // Initialize Protothreads
  // PT_INIT(&ptMotorControl);
  PT_INIT(&ptSetup);
  PT_INIT(&ptQuick);
  PT_SCHEDULE(SetupThread(&ptSetup));

  // PT_INIT(&ptUltrasound);
  // PT_INIT(&ptLights);
  // PT_INIT(&ptBatteryMonitor); // Initialize the Battery Monitor thread

  // Set up the sensor events
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

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
  delay(300);
  // Serial.println("Looping...");
  //  PT_SCHEDULE(MotorControlThread(&ptMotorControl));
  PT_SCHEDULE(Quickhread(&ptQuick));
  // PT_SCHEDULE(UltrasoundThread(&ptUltrasound));
  // PT_SCHEDULE(LightsThread(&ptLights));
  // PT_SCHEDULE(BatteryMonitorThread(&ptBatteryMonitor)); // Schedule Battery Monitor thread

  // Add more threads or logic as needed
}
