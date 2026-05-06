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
I am currentely using an external library made by barulicm (https://github.com/barulicm/PiPCA9685) and declare 2 variables `servo_driver` and `pwm_driver` in `PCA9685Driver.hpp`. They become defined in `Main.cpp` Learn more about the key methods used in [PCA9685Driver.hpp](#pca9685driverhpp)

### Camera.cpp

### Engine.cpp
Currentely, we are just have one method `SetThrust()` 

### GPS.cpp
Currentely, we only care about RMC and GGA NMEA sentecnes because those give us all the information we need, but if you ever need to add more, look at the datasheet `https://cdn-shop.adafruit.com/datasheets/PMTK_A11.pdf` and follow the current methods we hae defined for parsing


To Parse all of the NMEA sentences from Adafruit's GPS, we created a custom class. In the constructor, we initalize all of our instance variables
to default values and check to see if the GPS is actually plugged in and if not then we do not create the GPS log file that logs all of the GPS NMEA sentences. 

In `Navigation.cpp`, every time we each the end of the GPS's period which if the user ran `gps_setup.sh`, then the update frequency should be 100ms, it calls `update()`. If no GPS is connectded, the file descriptor will be negative, so we will always return. In every call to `update()`, we
set fresh_position and fresh_velocity to false so we do not add repeat data

`read_data()` is what actually reads the NMEA sentences from the buffer.  We only add complete messages which is what this line does because every NMEA sentence starts with `$`

```
  if (message.find('$') != std::string::npos)
 ```

Back in `update`(), after we call `read_data()`, we loop through all the NMEA senteces received from the buffer, and first we call
`break_message_down()` which creates a vector of all the NMEA sentences fields because every field is separated by a comma, then we call
`determine_NMEA_type()` to determine what NMEA type we parsed, and if it is either an RMC or GGA sentence, then we call `parse_NMEA_type()` where we
first make sure we have every field that is supposed to be there, the data is valid, and we are not checking the same time and if so, then we extract the data from it, reset accumulated_messages which is the variable that holds all of the complete NMEA messages recieved from the buffer, 
call `convert_coordinate_frame()` and increment update_count which is used in `Telemetry.cpp` to log the GPS navigation data that is calculated


### Igniter.cpp
NOTE: This was used in the past for igniting solid motor engines but now we are using a liquid fuled engine the
code might change

Currentely, there are 2 method `Ignite()` and `DisableIgnite()` which uses a `digitalWrite()` to whatever the ignition pin is which is set up in `MissionConstants.hpp`

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

Lidar is not currentely implemented

### LinActMotorPositionControl.cpp

`LinActMotorPositionControl.cpp` is used to control our linear actuators 

 The actuator that is used in the X direction is controlled using kTvcActuator0RpwmChannel and kTvcActuator0LpwmChannel const variables in MissionConstants
 These pins correspond to the PCA9685 pins


 The actuator that is used in the Y direction is controlled using kTvcActuator1RpwmChannel and kTvcActuator1LpwmChannel const variables in MissionConstants
These pins correspond to the PCA9685 pins


`moveToLimit()` is used to calibrate the Linear Actuators by extending them and retracting them and logging what the min and max potentiometer value is. We can calibrate the Linear Actuators by going into `ActuatorCalibration` Mode. After seeing the value is, replace the following MissionConstants variable values :
   - `kTvcActuator0PotentiometerMinReading`
   - `kTvcActuator0PotentiometerMaxReading`
   -  `kTvcActuator1PotentiometerMaxReading`
   - `kTvcActuator1PotentiometerMaxReading`

`driveActuator()` is the main function that actualy moves the actuators.
It takes in the `actuator_index, direction, and speed` where:
   - `actuator_index` corresponds to what actuator we are moving where 0 = actuator correspond to X direction and 1 =
   actuator correspond to Y direction
   - `direction` corresponds to either extending which is `1`, stopping `0` or retracting `-1`
   - `speed` corresponds to how fast the linear actuator moves and is directly correlated with the PWM signal generated from PCA9686
To actually move the actuator, we need to send 2 commands:
 -  pwm_driver->set_pwm(rpwmChannel, 0, speed);
 -  pwm_driver->set_pwm(lpwmChannel, 0, 0);
`pwm_driver` is the object fron the PCA9685 custom library and `set_pwm()` is the method we use to send the pwm signal. How the PCA works is documented under [PCA9685Driver.hpp](#pca9685driverhpp)
The important thing is to extend we send a non-zero pwm signal to rpwmChannel and 0 pwm signal to lpwmChannel, 
to retract we send a zero pwm signal to rpwmChannel and a non-zero pwm signal to lpwmChannel, and to stop we send
0 pwm signals to both rpwmChannel and lpwmChannel 

`readPositionInches()` uses the poteniometer readings to convert to a position measurement of the actuator

IMPORTANT:
   When we first got the linear actuator working, extended all the way read as 0V and retracted all the way read as 5V, but when setting up the test stand, speicifcally when wiring the potentiometer, we initially put 5V to GND and GND to 5v, but we fixed it, but now the readings are oppsoite than before. 


### LoadCell.cpp
`LoadCell` is used to measire the thrust generated by the engine. 
Our current cell can measure up to 1000kg and has a range of 0 to 10V. 
We use the ADS1115 to measure the analog signal where we then convert it to a voltage and then divide by 10 to get 
a percentage which we then multiply by 1000 to see how much kg it measured where we finally convert to lbs to measure that

IMPORTANT:
As of May 5th, 2026, the laod cell readings were not good 

### Magnometer.cpp
`Magnometer` is a part of the `IMU` so therefore it made sense to make `Magnometer.cpp` a child class from `IMU.cpp`. That means that `Magnometer's` Constructor defaults to `Imu's` constructor. The only method we had to define was `GetMagneticField()` which follows the same pattern as the Getter methods in IMU where we read from the Magnometer registers and then divide by the LSB convertion 

### PressureTransducer.cpp
`PressureTransducer` works the same as `LoadCell` where we first measure the anolog reading, convert to a voltage,
normalize it, and then mulitply by that pressure's max PSI value to convert it to a Pressure value 

### RF.cpp

### SparkPlug.cpp
`Sparkplug` is split into 3 methods `TurnOn()`, `TurnOff()`, and `IsOn()`

In `TurnOn()`, we first turn the Relay on by setting the Mission Constant `kSparkPin` to `LOW` and then we set a 
`2%` duty cycle to the 1000HZ PCA9685 through the Mission Constant `kRPMPin` pin

In `TurnOff()`, we first turn the Relay Off by setting the Mission Constant `kSparkPin` to `HIGH` and then we set a `0%` duty cycle to the 1000HZ PCA9685 through the Mission Constan `kRPMPin` pin

### TVC.cpp

### ValveControl.cpp
`ValveControl` is split into 3 method `OpenValve`, `ClosenValve`, `IsValveOpen`

In `OpenValve` and `CloseValve`, we first figure out how many of the 4095 ticks correspond to an Open position of 91 degrees which is based on the Mission Constant `kValveOpenAngle`.
To do so we use the equation:
```
float pulse = 1500 + ((angle - 90) / 90.0) * 1000;
return (pulse / MissionConstants::SERVO_PERIOD) * MissionConstants::MAX_TICKS;
```
Then, for the servo valves, we set use `servo_driver->set_pwm()` with the correct pin that is defined in `MissionConstants.hpp` 
For the relay valves, we use `digitalWrite()` with the correct pin that is defined in `MissionConstants.hpp` 

NOTE: `NitrogenBleed` Valve is weird because it is a normally open valve, when we call `OpenValve()` and `CloseValve`(), we are doing opposite of the other relay valves where for open, we send a HIGH signal and for close we send a LOW command

### PCA9685Driver.hpp 
Like I mentioned in [Introduction](#introduction), we use 2 PCA9685 boards, and because we are using the custom class made by someone else, all we need to do is initalize 2 objects from
his class, so to achieve that I created the `PCA9685Driver.hpp` file and while this file is under include, I'm tlaking about it here since a good majority of the hardware classes use them.

We use unique pointers here to delay the initilization of the variables until main, so we can check if the PCA9685's are connected in `Main.cpp`

The key methods used from barulicum `PCA6585.cpp` class are
 - `set_pwm()`
 - `set_pwm_freq()`

We use `set_pwm_freq()` to set the frequency of the PCA9685 which affects every channel, so we set the servo_driver variable to 50HZ for all the
servos and the pwm_driver to 1000HZ for all the linear actutors and rpmpin
We then use `set_pwm(const int channel, const uint16_t on, const uint16_t off)` to set the pwm signal to the specified channel.

The way setting a pwm signal works for the PCA9685 is that it has an internal clock that counts to 4095 so if we want to send a 100% duty cycle we set off to 4095 because it starts off high on 0 so when the count reaches all the way from to 4095, see we should turn off the pwm on signal now. Similarly, if we wanted a 50% duty cyle, we would just set off to 2048 so that we are ON until the count reaches 2048. 
