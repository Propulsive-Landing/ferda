# Table of Contents

1. [Introduction](#introduction)
   - [Mode.cpp](#modecpp)
   - [Navigation.cpp](#navigationcpp)
   - [Controller.cpp](#controllercpp)
   - [LaunchManager.cpp](#launchmanagercpp)
   - [Telemetry.cpp](#telemetrycpp)

## Introduction 
These are all of our non-hardware files.
### Mode.cpp
 
#### Summary
This is the brain of our Flight Software.
`Mode::Update()` is the key method where we determine what Mode method to call based on what Mode we are in.
We either change modes based on an RF command, a navigation condition, or a timing condition.
Whenever we do change modes, it is important that we use `Telemetry.GetInstance().Log()` so that we know we switched correctly.

#### How to Add More Modes
1. Go to `Mode.hpp` in the include/ directory
2. Add to the enum `Phase`
3. Under `private:`, add the new method for your mode following the naming convention of `Update...()`
4. Go back to `Mode.cpp` and implement the method
5. Add your Mode enum to the switch statement in `Update()`

#### Current Flow Diagram Implementation
![Mode Flow Diagram](../ModeFlowDiagram.png)

### Navigation.cpp

### Controller.cpp

### LaunchManager.cpp

### Telemetry.cpp
This class is in charge of writing all of our data to log files and sending our data to ground control through RF communication with XBees.

We call `RunTelemetry()` before executing the current Mode method.
In `RunTelemetry()`, we use `HardwareSaveDelta`, which controls how often we write to our log files, and `RFSaveDelta`, which controls how often we send data through RF. `gps_update_count` is not an argument but instead a local static variable that is used to control how often we write to the GPS log file.

NOTE:
We use the JSON data type to send data through RF, and we always append a newline because the RF.py class in ground control expects a newline when looking through the buffer.
