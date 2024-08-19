# ARD1

The ARD1 microcontroller is configured to manage various components essential for the autonomous robot car's operation. It handles motor control, sensor data acquisition, communication, and additional functionalities like lighting and data storage. The motor control is achieved using the TI SN754410NE H-Bridge Motor Driver, which governs four motors. Specifically, ARD1 uses the digital pins D3, D5, D7, and D9 to provide PWM signals to the motors, while pins D4, D6, D8, and D10 control the direction of each motor.

For monitoring motor speed, four KY-033 IR Infrared Sensors are connected to analog pins A0, A1, A2, and A3. These sensors provide real-time feedback on the rotational speed of each motor, enabling precise speed regulation and control. The RCWL-0516 Microwave Radar Sensor is integrated with ARD1 via digital pin D11, allowing the detection of motion in the robot’s surroundings. Additionally, the HC-SR04 Ultrasound Sensor is connected with its trigger and echo pins on D12 and D13, respectively. This sensor measures the distance to obstacles, crucial for pathfinding and collision avoidance.

A servo motor, used for rotating the ultrasound sensor to scan different directions, is controlled via digital pin D14. This setup allows for flexible positioning of the ultrasound sensor, enhancing the robot’s environmental awareness. ARD1 also includes a voltage monitoring feature using analog pin A6 to measure the battery’s voltage level, ensuring the robot can detect low power conditions.

Communication between ARD1 and other devices, such as the MPU6050 accelerometer and ARD2, is handled through the I2C protocol. The SDA (data line) and SCL (clock line) are connected to analog pins A4 and A5, respectively. This setup not only facilitates communication with the MPU6050 for navigation data but also allows future expansion of communication with ARD2, avoiding the use of D0 and D1, which are reserved for USB debugging.

For data storage, ARD1 interfaces with a SparkFun microSD Transflash Breakout Module. The module uses digital pins D15 (CS), D16 (MOSI), D17 (MISO), and D18 (SCK) to read from and write to the microSD card, storing maps and potentially images in the future. Finally, ARD1 controls the robot’s lighting system, with general lights connected to pin D19 and driving lights to pin D20. These lights can be turned on, off, or made to blink, providing visual feedback or signaling during operation.

This pin configuration ensures that ARD1 can effectively manage all necessary components, with room for future expansion through the I2C bus, while preserving the USB port for debugging purposes.

| **Component**                                       | **Function**           | **Pin on ARD1**       |
|-----------------------------------------------------|------------------------|-----------------------|
| **I2C Communication (MPU6050, ARD2)**               | SDA (Data Line)        | A4 (Pin 2)            |
|                                                     | SCL (Clock Line)       | A5 (Pin 3)            |
| **Motor Driver (TI SN754410NE)**                    | Motor 1 PWM            | D3 (Pin 9)            |
|                                                     | Motor 1 Direction      | D4 (Pin 8)            |
|                                                     | Motor 2 PWM            | D5 (Pin 10)           |
|                                                     | Motor 2 Direction      | D6 (Pin 12)           |
|                                                     | Motor 3 PWM            | D7 (Pin 13)           |
|                                                     | Motor 3 Direction      | D8 (Pin 14)           |
|                                                     | Motor 4 PWM            | D9 (Pin 15)           |
|                                                     | Motor 4 Direction      | D10 (Pin 16)          |
| **KY-033 IR Infrared Sensors**                      | Sensor 1               | A0 (Pin 7)            |
|                                                     | Sensor 2               | A1 (Pin 6)            |
|                                                     | Sensor 3               | A2 (Pin 5)            |
|                                                     | Sensor 4               | A3 (Pin 4)            |
| **RCWL-0516 Microwave Radar Sensor**                | Signal Pin             | D11 (Pin 17)          |
| **HC-SR04 Ultrasound Sensor**                       | Trigger                | D12 (Pin 18)          |
|                                                     | Echo                   | D13 (Pin 19)          |
| **Servo (for ultrasound rotation)**                 | Control Pin            | D14 (Pin 20)          |
| **Battery Voltage Monitoring**                      | Analog Pin             | A6 (Pin 19)           |
| **SparkFun microSD Transflash Breakout Module**     | CS                     | D15 (Pin 21)          |
|                                                     | MOSI                   | D16 (Pin 22)          |
|                                                     | MISO                   | D17 (Pin 23)          |
|                                                     | SCK                    | D18 (Pin 24)          |
| **Drive Lights**                                    | General Lights         | D19 (Pin 25)          |
|                                                     | Driving Lights         | D20 (Pin 26)          |
