# Table of Contents

1. [Introduction](#introduction)
   - [Camera.cpp](#cameracpp)
   - [Engine.cpp](#enginecpp)
   - [GPS.cpp](#gpscpp)
   - [Igniter.cpp](#ignitercpp)
   - [IMU.cpp](#imucpp)
   - [Lidar.cpp](#lidarcpp)
   - [LinActMotorPositionControl.cpp](#linactmotorpositioncontrolcpp)
   - [LoadCell.cpp](#loadcellcpp)
   - [Magnometer.cpp](#magnometercpp)
   - [PressureTransducer.cpp](#pressuretransducercpp)
   - [RF.cpp](#rfcpp)
   - [SparkPlug.cpp](#sparkplugcpp)
   - [TVC.cpp](#tvccpp)
   - [ValveControl.cpp](#valvecontrolcpp)



## Introduction 
These are all of our Hardware classes that are used in Release mode which should be used when the sensors are connected.

Our current hardware stack is:
1. 3 ADS1115 (https://www.ti.com/lit/ds/symlink/ads1115.pdf)
2. 2 Linear Actuators (https://www.firgelliauto.com/products/feedback-rod-actuator?variant=849524071)
3. 2 PCA9685s (https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf)
4. 1 BMO055 ((https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf))
5. 1 Load Cell
6. 1 Adafruit GPS (https://www.adafruit.com/product/4279?srsltid=AfmBOoqfHonDixBlvrF8NZrAJvkqtWSKWbYkq4jLcpdUHawGwJptTjV4)
7. 5 Servos
8. 2 Xbees
9. 1 Camera
10. 7 Pressure Transducers
11. 3 Solenoid Valves
12. 1 Relay Board
13. 1 Spark Plug

Tips:
1. On the raspberry pi, make sure you enabled I2C amd Serial Port on the pi 
   1. Run the command `sudo raspi-config` which will bring up the Configuration Tool GUI
   2. Use the Arrow keys to get to `Interface Options`, and then hit `Enter`
   3. Select `I2C`, and select `Yes` 
   4. Select `Serial Port` and select `Yes`
   5. Use the Arrow keys to select `Finish`
   6. Run the command `sudo reboot`
2. To check what I2C devices are currentely detected, run the command `i2cdetect -y 1`
   1. PCA9685 -> `0x40`
   2. PCA9685 -> `0x41` (The soldered one)
   You will also see `0x70` which is the `All Call` address that allows you send the same command to all PCA9685s
   3. BMO055 -> `0x28`
   4. ADS1115 -> `0x48` (ADDR is connected to GND which is the default)
   5. ADS1115 -> `0x49` (ADDR is connected to VDD)
   6. ADS1115 -> `0x4A` (ADDR is connected to SDA)

Notes:
The ADS1115 are our analog-to-digital-converters because the Pi doesn't have any Analog pins and they are used to measure all of our Pressure Transducers
We use `wiringPi's` `ads1115.c` custom library so we did not have to make our own and we initalize all our ads "variables" but they are not really variables with 
`ads1115setup()` in `Main.cpp`

The PCA9685 are our pwm drivers, and they are used to generate hardware PWM signals so we do not have to generate software generated ones which put a lot of strain on our CPU and are not as accurate 
1. We have one dedicated to 1000HZ which is what we use to drive our Linear Actuators and the Spark Plug RPM pin
2. The other one us dedicated to 50HZ PWM signals for all of our Servo Actuated Valves because Servos need 50Hz signals
I am currentely using an external library made by barulicm (https://github.com/barulicm/PiPCA9685) and declare 2 variables `servo_driver` and `pwm_driver` in `PCA9685Driver.hpp`. They become defined in `Main.cpp`



### Camera.cpp

### Engine.cpp

### GPS.cpp

### Igniter.cpp

### IMU.cpp
Currentely, we are using the BMO055 (https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf)

#### Setup
So, the IMU uses I2C as its communication protocol with the default address being `0x28`

It measures Acceleration, Angular Velocity, and Magnetic Field

In the datasheet, you will see that there are different modes that you can modes that IMU can be in

We want the Raw values, so we stick with AMG mode

In order to actually use the IMU, we do the following in the constructor:
1. We call `wiringPiI2CSetup()` with the IMU's I2C address which should be 0x28 which will open the Linux I2C device file which is most likely `/dev/i2c-1` and then, it sets the target slave address to `0x28`, and finally returns a file descriptor that represents the connection
2. Then, we do a quick check to see if BMO055 was connected by checking the chip_id 
3. Then, we set the Power Mode to Normal using `wiringPiI2cWriteReg8` with the second input is the Power Mode Register Address, and the third input is the Value we write to that register
4. Then we delay 10 seconds to the power mode to change
5. Then we set the operation mode to Config by again using `wiringPiI2CWriteReg8` with the secind input being the OPeration Mode Register Address and the third input being the Config Value
6. We delay 50 seconds to allow the changes to occur
7. Now, we set the operation mode to AMG
8. We delay 50 seconds to allow the changes to occur
9. Then we specify the Unit selection
10. We delay 50 seconds to allow the changes to occur

The pattern is you just need to find the register address you want to write to and then look at the differet values and once you have both you can use either `wiringPiI2cWriteReg8` to write the value or `wiringPiI2CReadReg8` to read the value the register

Note:
The way we read data is we first use `write` to set the chips internal register pointer
Then we use `read` to read in all 16 bytes where the we split them across a `uint8_t array` because the each data has 2 registers where one of them has the high 8 bits and the second has the low 8 bits and to read the full value we need to combine them. Using `read` and `write` is more efficent than `wiringPi's` library, but it is more convenient, so I left both options int

The last thing to note is for each data type, we need to divide by a number which accounts for bytes to the unit



### Lidar.cpp

### LinActMotorPositionControl.cpp

### LoadCell.cpp

### Magnometer.cpp

### PressureTransducer.cpp

### RF.cpp

### SparkPlug.cpp

### TVC.cpp

### ValveControl.cpp
