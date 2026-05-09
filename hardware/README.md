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
   - [Magnetometer.cpp](#magnetometercpp)
   - [PressureTransducer.cpp](#pressuretransducercpp)
   - [RF.cpp](#rfcpp)
   - [SparkPlug.cpp](#sparkplugcpp)
   - [TVC.cpp](#tvccpp)
   - [ValveControl.cpp](#valvecontrolcpp)



## Introduction 
These are all of our hardware classes that are used in Release mode, which should be used when the sensors are connected.

Our current hardware stack is:
1. 3 ADS1115 [ADS1115](../DatasheetsAndRPInfo/ads1115.pdf)
2. 2 Linear Actuators [Linear Actuator](https://www.firgelliauto.com/products/feedback-rod-actuator?variant=849524071)
3. 2 PCA9685s [PCA9685](../DatasheetsAndRPInfo/PCA9685.pdf)
4. 1 BNO055 [BNO055](../DatasheetsAndRPInfo/BMO055.pdf)
5. 1 Load Cell
6. 1 Adafruit GPS (https://www.adafruit.com/product/4279?srsltid=AfmBOoqfHonDixBlvrF8NZrAJvkqtWSKWbYkq4jLcpdUHawGwJptTjV4)
7. 5 25KG Servos 
8. 2 XBee ZB S2C [XBee ZB S2C](https://www.digi.com/support/knowledge-base/the-major-differences-in-the-xbee-series-1-vs-the)
9. 1 RPI Camera
10. 7 Pressure Transducers
11. 3 Solenoid Valves
12. 1 Relay Board
13. 1 Spark Plug

TIPS:
1. On the Raspberry Pi, make sure you enabled I2C and Serial Port on the Pi.
   1. Run the command `sudo raspi-config` which will bring up the Configuration Tool GUI
   2. Use the Arrow keys to get to `Interface Options`, and then hit `Enter`
   3. Select `I2C`, and select `Yes` 
   4. Select `Serial Port` and select `Yes`
   5. Use the Arrow keys to select `Finish`
   6. Run the command `sudo reboot`
2. To check what I2C devices are currently detected, run the command `i2cdetect -y 1`
   1. PCA9685 -> `0x40`
   2. PCA9685 -> `0x41` (The soldered one)
   You will also see `0x70` which is the `All Call` address that allows you send the same command to all PCA9685s
   3. BNO055 -> `0x28`
   4. ADS1115 -> `0x48` (ADDR is connected to GND which is the default)
   5. ADS1115 -> `0x49` (ADDR is connected to VDD)
   6. ADS1115 -> `0x4A` (ADDR is connected to SDA)

NOTES: 
On startup, when you run `./Ferda`, if any sensor is not connected, you will see a warning message issued by Telemetry.

The XBees are configured using XCTU software. When configuring them, the Ground Control XBee should be the Coordinator
and the Flight Computer XBee should be the End Device. The Coordinator is responsible for setting up the network. We also set the baud rate of both of them to 38400. To send data back and forth, the Channel and the Pan ID have to match.
Whenever you change the settings, you need to click on `Write`.

The ADS1115s are our analog-to-digital converters because the Pi doesn't have any analog pins, and they are used to measure all of our Pressure Transducers.
We use `wiringPi's` `ads1115.c` custom library so we did not have to make our own, and we initialize all our ADS "variables" with
`ads1115setup()` in `Main.cpp`

The PCA9685s are our PWM drivers, and they are used to generate hardware PWM signals so we do not have to generate software-generated ones, which put a lot of strain on our CPU and are not as accurate.
1. We have one dedicated to 1000 Hz, which is what we use to drive our Linear Actuators and the Spark Plug RPM pin.
2. The other one is dedicated to 50 Hz PWM signals for all of our Servo Actuated Valves because servos need 50 Hz signals.
I am currently using an external library made by barulicm (https://github.com/barulicm/PiPCA9685) and declare 2 variables, `servo_driver` and `pwm_driver`, in `PCA9685Driver.hpp`. They become defined in `Main.cpp`. Learn more about the key methods used in [PCA9685Driver.hpp](#pca9685driverhpp).

Troubleshooting:
If you see a segmentation fault error, it is most likely because of using `pwm_driver` or `servo_driver` when the sensors are not connected. We always assume that if we are building in
release mode, every sensor is attached, so for now you have to uncomment lines using them. An exception is in `LinActMotorPositionControl.hpp` in hardware; we have a null pointer check because it was used frequently in testing navigation sensors, so I left it in.
I also left it as it is because it is easier to debug. If the user decides to ignore the warning messages, then they will get a segfault, so they know something is wrong right away.

### Camera.cpp
`Camera.cpp` is used by Navigation to turn camera images into unit vectors that point from the camera to detected ground markers.

On startup, the camera tries to load calibration data from `../calibration/camera_calibration.json`. If that file is missing or invalid, it falls back to the built-in calibration constants. The calibration is used in `PixelToUnitVector()` with OpenCV's `cv::undistortPoints()` so the marker centroid pixels become normalized camera-frame rays.

`RequestCapture()` does not immediately take a picture. Instead, it marks a capture as pending and assigns the next frame ID. The next time Navigation calls `GetUnitVectorList()` or `GetFrameId()`, the camera runs `TryProcessPendingLocalCapture()`, captures the pending frame, detects markers, and updates the latest unit vector list.

The camera stream setup happens in `InitializeVideoStream()`. It first tries the preferred camera device index from `MissionConstants`, then tries other device indexes. Depending on the mission constants, it can use a native GStreamer pipeline or V4L2/OpenCV capture. After opening a stream, it reads warmup frames so the first processed frame is not stale or empty.

`CaptureLocalFrameAndProcess()` is the main image-processing method:
1. Captures a frame from OpenCV
2. Drops one buffered frame when possible so the result is recent
3. Detects white marker centroids using `WhiteCircle::DetectWhiteMarkerCentroids()`
4. Draws detection boxes and labels for debug frames
5. Converts each centroid into a camera-frame unit vector
6. Saves timing information and optional debug images

There are also two debug annotation helpers:
1. `AnnotateDebugFrameMatches()` labels which measured marker was matched to which expected marker
2. `AnnotateDebugFrameExpectedVsTrue()` overlays expected marker positions and actual detected positions

### Engine.cpp
Currently, we just have one method, `SetThrust()`.

### GPS.cpp
Currently, we only care about RMC and GGA NMEA sentences because those give us all the information we need, but if you ever need to add more, look at the datasheet [PMTK_A11.pdf](../DatasheetsAndRPInfo/PMTK_A11.pdf) and follow the current methods we have defined for parsing.


To parse all of the NMEA sentences from Adafruit's GPS, we created a custom class. In the constructor, we initialize all of our instance variables
to default values and check to see if the GPS is actually plugged in. If it is not, then we do not create the GPS log file that logs all of the GPS NMEA sentences.

In `Navigation.cpp`, every time we reach the end of the GPS's period, it calls `update()`. If the user ran `gps_setup.sh`, then the update frequency should be 100 ms. If no GPS is connected, the file descriptor will be negative, so we will always return. In every call to `update()`, we
set fresh_position and fresh_velocity to false so we do not add repeat data.

`read_data()` is what actually reads the NMEA sentences from the buffer.  We only add complete messages which is what this line does because every NMEA sentence starts with `$`

```
  if (message.find('$') != std::string::npos)
 ```

Back in `update()`, after we call `read_data()`, we loop through all the NMEA sentences received from the buffer, and first we call
`break_message_down()`, which creates a vector of all the NMEA sentence fields because every field is separated by a comma. Then we call
`determine_NMEA_type()` to determine what NMEA type we parsed, and if it is either an RMC or GGA sentence, then we call `parse_NMEA_type()` where we
first make sure we have every field that is supposed to be there, the data is valid, and we are not checking the same time. If so, then we extract the data from it, reset accumulated_messages, which is the variable that holds all of the complete NMEA messages received from the buffer,
call `convert_coordinate_frame()`, and increment update_count, which is used in `Telemetry.cpp` to log the GPS navigation data that is calculated.


### Igniter.cpp
NOTE:
This was used in the past for igniting solid motor engines, but now we are using a liquid-fueled engine, so the
code might change.

Currently, there are 2 methods, `Ignite()` and `DisableIgnite()`, which use a `digitalWrite()` to whatever the ignition pin is set to in `MissionConstants.hpp`.

### IMU.cpp
Currently, we are using the BNO055 ([BMO055.pdf](../DatasheetsAndRPInfo/BMO055.pdf)).

#### Setup
So, the IMU uses I2C as its communication protocol with the default address being `0x28`

It measures Acceleration, Angular Velocity, and Magnetic Field

In the datasheet, you will see that there are different modes that the IMU can be in.

We want the Raw values, so we stick with AMG mode

In order to actually use the IMU, we do the following in the constructor:
1. We call `wiringPiI2CSetup()` with the IMU's I2C address which should be 0x28 which will open the Linux I2C device file which is most likely `/dev/i2c-1` and then, it sets the target slave address to `0x28`, and finally returns a file descriptor that represents the connection
2. Then, we do a quick check to see if BNO055 was connected by checking the chip_id.
3. Then, we set the Power Mode to Normal using `wiringPiI2cWriteReg8`, with the second input as the Power Mode Register Address and the third input as the value we write to that register.
4. Then we delay 10 milliseconds for the power mode to change.
5. Then we set the operation mode to Config by again using `wiringPiI2CWriteReg8`, with the second input being the Operation Mode Register Address and the third input being the Config Value.
6. We delay 50 milliseconds to allow the changes to occur.
7. Now, we set the operation mode to AMG
8. We delay 50 milliseconds to allow the changes to occur
9. Then we specify the Unit selection
10. We delay 50 milliseconds to allow the changes to occur

The pattern is that you just need to find the register address you want to write to and then look at the different values. Once you have both, you can use either `wiringPiI2cWriteReg8` to write the value or `wiringPiI2CReadReg8` to read the value of the register.

NOTE:
The way we read data is we first use `write` to set the chips internal register pointer
Then we use `read` to read in all 16 bytes, where we split them across a `uint8_t array` because each data value has 2 registers. One of them has the high 8 bits and the second has the low 8 bits, and to read the full value we need to combine them. Using `read` and `write` is more efficient than `wiringPi's` library, but it is more convenient, so I left both options in.

The last thing to note is for each data type, we need to divide by a number which accounts for bytes to the unit

### Lidar.cpp

As of May 7th, 2026, Lidar is not currently hooked up to the Pi, so we have not implemented it fully.

### LinActMotorPositionControl.cpp

`LinActMotorPositionControl.cpp` is used to control our linear actuators 

The actuator that is used in the X direction is controlled using the kTvcActuator0RpwmChannel and kTvcActuator0LpwmChannel const variables in MissionConstants.
These pins correspond to the PCA9685 pins.


The actuator that is used in the Y direction is controlled using the kTvcActuator1RpwmChannel and kTvcActuator1LpwmChannel const variables in MissionConstants.
These pins correspond to the PCA9685 pins.


`moveToLimit()` is used to calibrate the Linear Actuators by extending and retracting them and logging the min and max potentiometer values. We can calibrate the Linear Actuators by going into `ActuatorCalibration` mode. After seeing what the value is, replace the following MissionConstants variable values:
   - `kTvcActuator0PotentiometerMinReading`
   - `kTvcActuator0PotentiometerMaxReading`
   -  `kTvcActuator1PotentiometerMinReading`
   - `kTvcActuator1PotentiometerMaxReading`

`driveActuator()` is the main function that actually moves the actuators.
It takes in the `actuator_index, direction, and speed` where:
   - `actuator_index` corresponds to what actuator we are moving, where 0 = actuator corresponding to X direction and 1 = actuator corresponding to Y direction
   - `direction` corresponds to either extending, which is `1`, stopping, which is `0`, or retracting, which is `-1`
   - `speed` corresponds to how fast the linear actuator moves and is directly correlated with the PWM signal generated from PCA9685
To actually move the actuator, we need to send 2 commands:
 -  pwm_driver->set_pwm(rpwmChannel, 0, speed);
 -  pwm_driver->set_pwm(lpwmChannel, 0, 0);
`pwm_driver` is the object from the PCA9685 custom library, and `set_pwm()` is the method we use to send the PWM signal. How the PCA works is documented under [PCA9685Driver.hpp](#pca9685driverhpp).
The important thing is that to extend, we send a non-zero PWM signal to rpwmChannel and a 0 PWM signal to lpwmChannel.
To retract, we send a zero PWM signal to rpwmChannel and a non-zero PWM signal to lpwmChannel, and to stop we send
0 PWM signals to both rpwmChannel and lpwmChannel.

`readPositionInches()` uses the potentiometer readings to convert to a position measurement of the actuator.

IMPORTANT:
   When we first got the linear actuator working, extended all the way read as 0V and retracted all the way read as 5V, but when setting up the test stand, specifically when wiring the potentiometer, we initially put 5V to GND and GND to 5V. We fixed it, but now the readings are opposite from before.


### LoadCell.cpp
`LoadCell` is used to measure the thrust generated by the engine.
Our current cell can measure up to 1000kg and has a range of 0 to 10V. 
We use the ADS1115 to measure the analog signal where we then convert it to a voltage and then divide by 10 to get 
a percentage, which we then multiply by 1000 to see how many kg it measured, where we finally convert to lbs to measure that.

IMPORTANT:
As of May 5th, 2026, the load cell readings were not good.

### Magnetometer.cpp
`Magnetometer` is a part of the `IMU`, so it made sense to make `Magnetometer.cpp` a child class from `IMU.cpp`. That means that `Magnetometer's` constructor defaults to `Imu's` constructor. The only method we had to define was `GetMagneticField()`, which follows the same pattern as the getter methods in IMU where we read from the Magnetometer registers and then divide by the LSB conversion.

### PressureTransducer.cpp
`PressureTransducer` works the same as `LoadCell`, where we first measure the analog reading, convert it to a voltage,
normalize it, and then multiply by that pressure's max PSI value to convert it to a pressure value.

### RF.cpp
We are currently using 2 XBee ZB S2C series, which should have an outdoor line-of-sight range of 4000ft. [XBee ZB S2C](https://www.digi.com/support/knowledge-base/the-major-differences-in-the-xbee-series-1-vs-the).

In the constructor, we first check to see if the XBee is connected, and if it is not, then we just use the terminal to send commands like how we would for user input. However, if the XBee is connected, then we set some serial settings, with the most important one being the baud rate to 38400 to match what the other XBee on Ground Control should be.

There are 3 methods at play here:
`ParseCommand()` which is defined in `RF.hpp`
`SendString()`
`GetCommand()`

`ParseCommand()` takes in a string and sees if any of the commands the user typed in Ground Control matches any already defined commands. If so, it returns that specified command.

`SendString()` is only used if the XBee is connected, and it makes sure it writes the entire message to the other XBee.

`GetCommand()` reads the data sent by the Ground Control XBee and waits until it sees a newline because that signifies the end of a message. When that happens, we call `ParseCommand()` to see if it is a valid command. If the XBee is not connected, we constantly look to see if the user inputted a command in the terminal.

## How to add a New command
1. Go To RF.hpp and add the name of the command in the Command enum
2. In `ParseCommand()`, add another else if statement where if the user entered the command, then we set `ParsedCommand` to that command enum value

## Current RF Commands
Mode commands:
1. `Calibration`
2. `Idle`
3. `Standby`
4. `Ignite`
5. `ABORT`
6. `ABORT_PAD`
7. `ABORT_GROUND`
8. `HotfireIdle`

TVC and navigation commands:
1. `TestTVC`
2. `ChirpTVC`
3. `StopTVC`, `STOP`, or `stop`
4. `CenterTVC`, `CENTER`, or `center`
5. `IncrementXTVC`
6. `IncrementYTVC`
7. `DecrementXTVC`
8. `DecrementYTVC`
9. `NAV_RESTART`
10. `ActuatorCalibration`, `ActuatorCalibrate`, or `ACTUATOR_CALIBRATION`
11. `MoveXTVCToLimitExtend`, `MoveXToLimitExtend`, or `MOVE_X_TO_LIMIT`
12. `MoveXTVCToLimitRetract` or `MoveXToLimitRetract`
13. `MoveYTVCToLimitExtend`, `MoveYToLimitExtend`, or `MOVE_Y_TO_LIMIT`
14. `MoveYTVCToLimitRetract` or `MoveYToLimitRetract`

Sensor toggle commands:
1. `SENSOR: camera ON`
2. `SENSOR: camera OFF`
3. `SENSOR: lidar ON`
4. `SENSOR: lidar OFF`
5. `SENSOR: gps_velocity ON`
6. `SENSOR: gps_velocity OFF`
7. `SENSOR: gps_position ON`
8. `SENSOR: gps_position OFF`
9. `SENSOR: magnetometer ON`
10. `SENSOR: magnetometer OFF`

Liquid propulsion manual commands:
1. `VALVE: nitrogen open`
2. `VALVE: nitrogen close`
3. `VALVE: purge open`
4. `VALVE: purge close`
5. `VALVE: main ethanol open`
6. `VALVE: main ethanol close`
7. `VALVE: main nitrous open`
8. `VALVE: main nitrous close`
9. `VALVE: nitrous fill open`
10. `VALVE: nitrous fill close`
11. `VALVE: ASI ethanol open`
12. `VALVE: ASI ethanol close`
13. `VALVE: ASI oxygen open`
14. `VALVE: ASI oxygen close`
15. `VALVE: nitrogen bleed open`
16. `VALVE: nitrogen bleed close`
17. `SPARK: on`
18. `SPARK: off`

Liquid propulsion sequence commands:
1. `asitest`
2. `waterflow`
3. `3second`

NOTE:
`LidarOn` and `LidarOff` exist in `RF.hpp`, but as of now `Mode.cpp` does not handle those commands in `CheckForToggleSensorCommands()`.

IMPORTANT:
As of May 6th, 2026, in Ground Control, when we are parsing the json payload sometimes we get a parsing error like 
```
Received: Unexpected serial parsing error (Expected 9 engine values or 30 GNC values, got 15): {'data_type': 'telem', 'payload': [9.49, 0.0, 0.0, 0.35, 147.29, 755.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'type': 'GNC'} time: 125.23
Received: Unexpected serial parsing error (Expected 9 engine values or 30 GNC values, got 24): {'data_type': 'telem', 'payload': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.43, 12.82, 4.14, 124.34], 'type': 'Liquid'} time: 190.24
```
I think this is because of how the XBees are sending data because currently they are in AT mode, which is Transparent mode
and does not have any structure. Please investigate further.


### SparkPlug.cpp
`SparkPlug` is split into 3 methods: `TurnOn()`, `TurnOff()`, and `IsOn()`.

In `TurnOn()`, we first turn the Relay on by setting the Mission Constant `kSparkPin` to `LOW` and then we set a 
`2%` duty cycle to the 1000 Hz PCA9685 through the Mission Constant `kRPMPin` pin.

In `TurnOff()`, we first turn the Relay Off by setting the Mission Constant `kSparkPin` to `HIGH` and then we set a `0%` duty cycle to the 1000 Hz PCA9685 through the Mission Constant `kRPMPin` pin.

### TVC.cpp
`TVC.cpp` is in charge of converting commanded TVC gimbal angles into linear actuator lengths and then driving the actuators to those lengths.

The main command path is:
1. `Controller.cpp` calls `SetTVCX()` and `SetTVCY()` with desired gimbal angles in radians
2. `SetTVCX()` and `SetTVCY()` add the compile-time trim values from `MissionConstants` and clamp the command to `kMaximumTvcAngle`
3. `UpdateActuatorPositions()` converts the stored angles into desired actuator lengths
4. The current actuator lengths are read using `readPositionInches()`
5. `ProportionalPositionControl()` computes the direction and speed command for each actuator
6. `driveActuator()` sends the final command to the PCA9685 motor channels

`AnglesToActuatorLengths()` does the TVC geometry. It uses the vehicle-side and engine-side mount points from `MissionConstants.hpp`, builds a rotation matrix from the commanded x/y gimbal angles, rotates the engine mount points, and calculates the actuator extension lengths. The resulting lengths are clamped between `kTvcMinLengthInches` and `kTvcMaxLengthInches`.

`ProportionalPositionControl()` is really a PID-style position controller, but right now the integral and derivative gains are set by the constants in `MissionConstants.hpp`. The output is a desired actuator velocity in inches per second. That velocity gets converted into:
   - `direction = 1` for extend
   - `direction = -1` for retract
   - `direction = 0` for stop
   - `speed_cmd`, which is clamped from 0 to `kTvcMaxMotorSpeed`

`Stop()` resets the stored TVC angles, clears the controller integral and previous error terms, zeroes the last speed commands, and commands both actuators to stop.

Telemetry logs a row to the actuator log every time `UpdateActuatorPositions()` runs. The row includes commanded angles, desired actuator lengths, observed actuator lengths, and the x/y speed commands.

IMPORTANT:
The TVC constants in `MissionConstants.hpp` are pre-flight calibration values. Before using TVC with hardware, check the mount point geometry, actuator min/max readings, actuator channel assignments, center trims, and maximum command limits.

### ValveControl.cpp
`ValveControl` is split into 3 methods: `OpenValve`, `CloseValve`, and `IsValveOpen`.

In `OpenValve` and `CloseValve`, we first figure out how many of the 4095 ticks correspond to an open position of 91 degrees, which is based on the Mission Constant `kValveOpenAngle`.
To do so we use the equation:
```
float pulse = 1500 + ((angle - 90) / 90.0) * 1000;
return (pulse / MissionConstants::SERVO_PERIOD) * MissionConstants::MAX_TICKS;
```
Then, for the servo valves, we use `servo_driver->set_pwm()` with the correct pin that is defined in `MissionConstants.hpp`.
For the relay valves, we use `digitalWrite()` with the correct pin that is defined in `MissionConstants.hpp`.

NOTE: `NitrogenBleed` Valve is weird because it is a normally open valve. When we call `OpenValve()` and `CloseValve()`, we are doing the opposite of the other relay valves: for open, we send a HIGH signal, and for close, we send a LOW command.

### PCA9685Driver.hpp 
Like I mentioned in [Introduction](#introduction), we use 2 PCA9685 boards, and because we are using the custom class made by someone else, all we need to do is initialize 2 objects from
his class. To achieve that, I created the `PCA9685Driver.hpp` file, and while this file is under include, I'm talking about it here since a good majority of the hardware classes use them.

We use unique pointers here to delay the initialization of the variables until main, so we can check if the PCA9685s are connected in `Main.cpp`.

The key methods used from barulicm's `PCA9685.cpp` class are:
 - `set_pwm()`
 - `set_pwm_freq()`

We use `set_pwm_freq()` to set the frequency of the PCA9685, which affects every channel, so we set the servo_driver variable to 50 Hz for all the
servos and the pwm_driver to 1000 Hz for all the linear actuators and rpmpin.
We then use `set_pwm(const int channel, const uint16_t on, const uint16_t off)` to set the pwm signal to the specified channel.

The way setting a PWM signal works for the PCA9685 is that it has an internal clock that counts to 4095, so if we want to send a 100% duty cycle, we set off to 4095 because it starts off high on 0. When the count reaches 4095, we should turn off the PWM signal. Similarly, if we wanted a 50% duty cycle, we would just set off to 2048 so that we are ON until the count reaches 2048.
