# Ferda :rocket:

Holds all flight software for the UConn Propulsive Landing team rockets. :smile:

All source code utilizes [Hungarian Notation](https://www.cse.iitk.ac.in/users/dsrkg/cs245/html/Guide.htm).

Create features in branches originating from the `dev` branch. When a feature is complete, make a pull request to merge it into `dev`.


## Table of Contents

1. [How to Run](#how-to-run)
   - [How to Run On Linux (Raspberry Pi)](#how-to-run-on-linux-raspberry-pi)
      - [Connecting to Raspberry Pi](#connecting-to-raspberry-pi)
   - [How to Run On Mac](#how-to-run-on-mac)
   - [How to Run On Windows](#how-to-run-on-windows)
   - [Troulbleshooting](#troubleshooting)
2. [Building the Source Code](#building-the-source-code)
3. [Hardware Configuration](#hardware-configuration)
   - [Xbee Port Setup](#xbee-port-setup)
   - [GPS Port Script](#gps-port-setup)
4. [Software-in-the-Loop Testing](#software-in-the-loop-testing)
   - [SIL Testing Using Windows + WSL2](#sil-testing-using-windows--wsl2)
   - [SIL Testing Using Windows](#sil-testing-using-windows)
5. [Design](#design)
   - [Flow](#flow)
   - [Architectire](#architecture)

# How to Run 

### How to Run on Linux (Raspberry Pi)

1. Install git (https://git-scm.com/install/)
2. Clone this repo.
3. Install the CMake Tools extension on VS Code.
4. Run `sudo apt update`
5. Run `xargs -a linux_library_requirements.txt sudo apt-get install -y`
6. Run `mkdir ThirdPartyLibraries` and then `cd ThirdPartyLibraries`
7. Clone WiringPi library with `git clone https://github.com/WiringPi/WiringPi.git` 
8. Run `cd WiringPi/wiringPi`
9. Run `make`
10. Run `sudo make install`
11. Go back to `ThirdPartyLibraries` directory
12. Clone PCA9685 library with `git clone https://github.com/barulicm/PiPCA9685.git`
13. Run `cd PiPCA9685`
14. Run `sudo cmake --workflow --preset install`
15. Build the repo (ensure you're in either debug, simulation, or release mode depending on your need).
16. Create a logs folder inside repo directory with `mkdir logs`
17. Run the executable that gets created in the `build/` folder. (NOTE: If working with GPS, run gps_setup.sh first)

#### Connecting to Raspberry Pi
In order to connect to the Raspberry Pi, it needs to be connected to wifi, and you need to know the IP address. 
All 3 raspberry Pi's including the 2 RPI5's and the 1 RPI4 are autimatically configured to connect to the TP-Link Archer C54 router that
we bought. Once you know the IP adress, then run the command
`ssh host@{IP-Adress}` where host is the pi's username and the IP address is the ip address the rpi is connected to 

Troubleshooting:
- Make sure you are connected to the same wifi as the Pi.
- If you used the same ip adress as another pi, it will yell at you and in that case run the commad `ssh-keygen -R {IP-address}` 

### How to Run on Mac
1. Install git (https://git-scm.com/install/)
2. Clone this repo 
3. Install brew with `/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"`
4. Run `brew bundle install`
5. Build the repo (ensure you're in either debug, simulation, or release mode depending on your need).
6. Create a logs folder inside repo directory with `mkdir logs`
7. Run the executable that gets created in the `build/` folder. (NOTE: If working with GPS, run gps_setup.sh first)

### How to Run on Windows

### Troubleshooting
- Make sure all dependencies are installed
   - linux_library_requirements.txt for linux operating systems (Raspberry Pi)
   - Brewfile for mac

- If you get `warning depend.make has modification time` issue:
	try the commamnds: make clean
                      make all

- When SSHing into the Pi, the IP address might change so go to the router's ip address to look at the exact the IP adress the Pi is connected to

- On mac, if you installed `CMake Tools extension` on VS Code and you have `ninja` installed, it might try to autimatically select it so when you try to build with `make`,
it will throw an error

## Building the Source Code

1. Clone the repository:
   ```bash
   git clone https://github.com/Propulsive-Landing/ferda.git
   ```

2. Enter the new folder:
   ```bash
   cd ferda
   ```

3. Generate the build files using CMake:
   - For release:
     ```bash
     cmake -Bbuild -DCMAKE_BUILD_TYPE=Release .
     ```
     (this uses the actual hardware sensors)
   - For debug:
     ```bash
     cmake -Bbuild -DCMAKE_BUILD_TYPE=Debug .
     ```
   - For Simulation:
     ```bash
     cmake -Bbuild -DCMAKE_BUILD_TYPE=Simulation .
     ```

4. Enter the newly generated `build` folder:
   ```bash
   cd build
   ```

5. Build the source:
   ```bash
   make all
   ```

6. Run the executable inside the `build` folder:
   ```bash
   sudo ./Ferda
   ```

7. Create a `logs` folder in the root of the project (without it, no logs will be saved).

## Hardware Configuration

### Xbee Port Setup

The Xbee module is a radio module which is used by our flight computer to send and receive data from the ground control, such as when we want to instruct the rocket to launch, or when we want to tell it to abort. To use the Xbee from our Raspberry Pi, we must configure it properly, here's how:

1. Xbee is currently (4/24/2026) configured to act as a terminal that only outputs values when there is a newline character.
2. In hardware/RF.cpp, we configure serial port settings to most importantly have it be baud 38400, but make sure that the Xbees are configured properly using XCTU software
3. For debugging, use `stty -F /dev/{RFPort} -a` to see all serial line settings because
   most of the time, the baud rate is not set to what the other XBee is or echo is not turned off
2. Use `stty` to configure the device:
   ```bash
   stty -F /dev/ttyUSB0
   ```
   (`/dev/ttyUSB0` may change per device).
3. Configure settings:
   ```bash
   stty -F /dev/ttyS0 -settingToDisable settingToEnable
   ```
   Disable settings using a minus sign and enable settings without it.
4. The device configuration should resemble:
   ```
   speed 3800 baud; line = 0;
   -echo
   ```

### GPS Port Setup
We use Adafruit's GPS module with a USBC port to help with our navigation, but first we need
to configure the GPS to specific settings. Every GPS receiver uses NMEA sentences which are
ASCII text strings that display information such as position, speed, and time. The ones that we care about are:
   - RMC (Recommended Minimum Specific GNSS Data)
   - GGA (Global Positioning System Fix Data)
To write settings, we use the talker ID `PMTK`. In every PMTK sentence we write, we need to compute the checksum which is the xors of every bit and we need o end with `\r\n`
There are steps to configure which are taken care of in `gps_setup.sh`
1. In `gps_setup.sh`, we call `gps_startup.py` to make sure the GPS has fix because in my testing, the settings only applied if there was a fix and there are nice libraries dealing witb Adafruit GPS in python 
2. Then we configure the baud rate to 9600 and disable echo
3. Then we use printf to write the setting, `$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n'` which enables which sentences we want 
4. Then we use printf to write the setting, `$PMTK251,38400*27\r\n'` which changes the baud rate to 38400
5. Then we need to change the baud rate for our serial port so we do `stty -F "$PORT" 38400 raw -echo -ixon`
6. Lastly, we change the update rate to be 10HZ using `'$PMTK220,100*2F\r\n'`

Documentation on PMTK command packets can be at `https://cdn-shop.adafruit.com/datasheets/PMTK_A11.pdf` 
Documentation on the NMEA sentences can be found at `https://cdn-shop.adafruit.com/product-files/746/CD+PA1616S+Datasheet.v03.pdf`


## Camera Calibration

The camera can now load a persisted OpenCV calibration file at startup. By default it looks for `../calibration/camera_calibration.json` when the executable is run from the `build/` directory.

1. Capture a set of checkerboard images from the camera at the same resolution used for flight.
2. Build the calibration tool and run it from the `build/` directory:
   ```bash
   ./CameraCalibrate ../calibration_images ../calibration/camera_calibration.json 9 6 0.024
   ```
   Replace `9 6` with the checkerboard inner-corner count and `0.024` with the square size in meters.
3. Restart Ferda. If the calibration file is present and valid, the camera uses it for `cv::undistortPoints()`; otherwise it falls back to the current built-in constants.

The calibration output stores the camera matrix, distortion coefficients, image size, checkerboard dimensions, and RMS reprojection error. Keep one file per physical camera if the hardware changes.

## Software-in-the-Loop Testing

Software-in-the-loop (SIL) testing allows you to connect your flight software to MATLAB's Simulink environment for real-time simulation. Follow the steps below to set up and run the SIL testing environment.

### SIL Testing Using Windows + WSL2

#### Install WSL2 - [help](https://gcore.com/learning/how-to-install-wsl-2-on-windows/)

To run your flight software in a Linux environment on Windows, you need to install WSL2 (Windows Subsystem for Linux). The steps can varry depending on your setup so you may have to do some troubleshooting to install it properly. Follow these steps:

1. Open PowerShell as Administrator and run:
   ```bash
   wsl --install
   ```

2. Set up your preferred Linux distribution (e.g., Ubuntu) as your default WSL instance.

3. Ensure WSL2 is set as the default version:
   ```bash
   wsl --set-default-version 2
   ```

4. Once your distribution is set up, enter your WSL terminal and update your package list:
   ```bash
   sudo apt update
   ```

#### Install Required Tools

To build the flight software, you need to install several tools in your WSL environment:

1. **CMake** – for managing the build process:
   ```bash
   sudo apt install cmake
   ```

2. **Make** – a build automation tool:
   ```bash
   sudo apt install build-essential
   ```

3. **Clang** – a C++ compiler:
   ```bash
   sudo apt install clang
   ```

4. **Git** – to clone and manage your repositories:
   ```bash
   sudo apt install git
   ```

5. **GitHub CLI (gh)** – for interacting with GitHub:
   ```bash
   sudo apt install gh
   ```

#### Authenticate with GitHub

To interact with private GitHub repositories and push code, you need to authenticate with GitHub CLI:

1. Log in using the GitHub CLI:
   ```bash
   gh auth login
   ```

2. Follow the prompts to authenticate via a web browser or with a GitHub token. Choose **HTTPS** as the protocol when prompted.

#### Get the Host and WSL IP Addresses

1. To get the **host machine's IP**:
   Open a WSL terminal and run:
   ```bash
   ip route show | grep -i default | awk '{ print $3}'
   ```
   Look for the IP address under the "Ethernet adapter vEthernet (WSL)" section.

2. To get the **WSL instance's IP**:
   In your WSL terminal, run:
   ```bash
   hostname -I
   ```

#### Clone and Build the Flight Software

1. Clone the flight software repository in your WSL terminal and switch to the correct branch (likely dev):
   ```bash
   git clone https://github.com/Propulsive-Landing/ferda.git
   cd ferda
   git switch dev
   ```

2. Build the software in simulation mode using the:
   ```bash
   cmake -DCMAKE_BUILD_TYPE=Simulation -DSIM_LOCAL_PORT=8002 -DSIM_SERVER_PORT=8003 -DSIM_SERVER_IP="[Insert Host IP here]" -Bbuild .
   cd build
   make all
   ```
   Note: Ports may change in future versions of simulation.

#### Configure the Simulink Model

1. In MATLAB, open the Simulink model for flight simulation.
2. Set the **UDPSend** block to the WSL IP and port 8002, and ensure the **UDPReceive** is set to receive from any source.
3. Install **Simulink Desktop Real-Time** to run the simulation in real-time using the Add-on manager in Matlab
4. Make sure you have the Real-Time Kernel installed. A guide on this is shown [here](https://www.mathworks.com/help/sldrt/ug/real-time-windows-target-kernel.html)
5. Set Simulink to **Connected IO** mode and start the simulation.

#### Run the Flight Software

Switch back to WSL and run the flight software:
```bash
sudo ./Ferda
```

### ~~SIL Testing Using Windows~~ (NOT TESTED)

For users who prefer running the flight software directly on Windows without WSL, you can follow this guide to set up and run the simulation loop using Windows native tools and loopback IP. However, please note that the flight computer uses a linux based OS and therefore, discrepancies may occur.

#### Install the Required Tools

To build and run the flight software natively on Windows, you'll need to install several development tools:

1. **CMake** – Download and install from the official website: [CMake](https://cmake.org/download/).
   - Make sure to add CMake to your system path during installation.

2. **Visual Studio (or Build Tools for Visual Studio)** – This will provide a C++ compiler. You can install the **Desktop development with C++** workload via the Visual Studio Installer or download **Build Tools for Visual Studio** from [here](https://visualstudio.microsoft.com/visual-cpp-build-tools/).
   - Ensure that the "MSVC" compiler and CMake support are selected during installation.

3. **Git** – Download and install Git from the [Git website](https://git-scm.com/download/win). This will allow you to clone the repository and manage version control.

4. **GitHub CLI (gh)** – Install GitHub CLI by downloading it from [GitHub](https://cli.github.com/). Once installed, authenticate via:
   ```bash
   gh auth login
   ```
   Follow the prompts to authenticate with your GitHub account.

#### Clone and Build the Flight Software

1. **Clone the Repository:**
   Open a Git Bash or command prompt on Windows and clone the repository:
   ```bash
   git clone https://github.com/Propulsive-Landing/ferda.git
   cd ferda
   git switch dev
   ```

2. **Generate the Build Files Using CMake:**
   In your Git Bash, command prompt, or terminal of choice, configure the build for simulation mode:
   ```bash
   cmake -Bbuild -DCMAKE_BUILD_TYPE=Simulation -DSIM_LOCAL_PORT=8002 -DSIM_SERVER_PORT=8003 -DSIM_SERVER_IP="127.0.0.1"
   ```

   Explanation of flags:
   - `-DCMAKE_BUILD_TYPE=Simulation`: This flag sets the build for simulation.
   - `-DSIM_LOCAL_PORT=8002`: Port used to receive data from Simulink.
   - `-DSIM_SERVER_PORT=8003`: Port to send data to Simulink.
   - `-DSIM_SERVER_IP="127.0.0.1"`: Since we are running everything on the same machine, use `127.0.0.1` (loopback IP).

3. **Compile the Software:**
   Navigate to the build directory and compile the software:
   ```bash
   cd build
   cmake --build .
   ```

   This will generate the `Ferda.exe` executable file in the build folder.

4. **Create a Logs Directory (Optional):**
   If logging is enabled, make sure you have a `logs` directory in the root of the project:
   ```bash
   mkdir logs
   ```

#### Configure the Simulink Model

1. **Launch MATLAB and Simulink:**
   Open the Simulink model for flight simulation in MATLAB.

2. **Configure UDPSend/UDPReceive Blocks:**
   - Set the **UDPSend** block to send data to the loopback IP (`127.0.0.1`) on port `8002`.
   - Ensure that the **UDPReceive** block is configured to listen on port `8003`.

3. **Install Simulink Desktop Real-Time:**
   You need the **Simulink Desktop Real-Time** add-on to run simulations in real-time. You can install this via the Add-On Manager in MATLAB.

4. **Set Simulink to Connected IO Mode:**
   - Set Simulink to **Connected IO** mode to simulate real-time hardware.

5. **Start the Simulation:**
   Start the Simulink simulation. 
   - The rocket should remain static until your flight software commands it.
   - Make sure your model is running in real-time with **1:1 real-time to simulation**

#### Run the Flight Software

1. **Return to the Command Prompt:**
   Switch back to your command prompt or Git Bash.

2. **Run the Flight Software:**
   Start the flight software by running:
   ```bash
   ./Ferda.exe
   ```

Notes:
- Run ferda (FSW) through WSL first if you're having trouble communicating with the simulation
- WSL can be tricky to initially set up but there is plenty of available documentation to resolve these issues, start [here](https://gcore.com/learning/how-to-install-wsl-2-on-windows/) and use google if you still are having issues.
  
The flight software should be connected with the Simulink simulation, sending actuator commands and receiving simulated sensor data.

# Design
We have separated all of our header files in the `include` directory, all non-hardware files into `src` firectory and all hardware files into the `hardware` directory. Now depending on the build type that you used, we either use `hardware` if built in `Release` mode, `hardware_test` if built in `Debug` Mode, or `hardware_simulation` if built in `Simulation` mode.

README'S for `Src` and `Hardware` files are located in [Src files README](Src/README.md) and [Hardware files README](hardware/README.md) 

If you add a new sensor, it belongs in the 3 hardware directories

All config files listed below are in the ferda directory but should at somepoint be in a Config folder
 - Angles.csv
 - Height.csv
 - Translation.csv 
 - linux_library_requirements.txt
 - Brewfile

We do have a `tests` folder but it has not been touched in a while. If you generate any unit tests, it should go here

For all our GPIO pin habdling, we have switched over to `WiringPi` because it was the easiest library to use.
For more information about Raspberry Pi's GPIO look here [GPIO](<DatasheetsAndRPInfo/RP-006553-WP-2-A history of GPIO usage on Raspberry Pi devices, and current best practices.pdf>)

For all our PWM handling, we wanted to use hardware PWM signals but depending on which Raspberry Pi we are using, it can have a maximum of 4 hardwre PWM GPIO pins, so we decided to buy PCA9685 boards which can generate up to hardware GPIO signals. For more information about the PCA9685 look here [PCA9685](DatasheetsAndRPInfo/PCA9685.pdf)

## Flow
1. Look for command through RF.cpp
2. Change mode based on command

## Architecture

We use the **State machine pattern** where we have defined several modes/states and one while loop in main. We use RF
to change modes but modes can also change based on a condition in the code as well.

In Mode.hpp, we define the Modes:
```
enum Phase
    {
        Standby,
        Calibration,
        ActuatorCalibration,
        TestTVC,
        ChirpTVC,
        Idle,
        Launch,
        Land,
        Terminate,
        Abort,
        Safe,
        HotfireIdle,
        ASITest,
        WaterFlow,
        ThreeSecondHotfire
    };
```

In RF.hpp, a subset of the commands look like below where Command is an enum where we add the RF command
```
 if (input_line == "ABORT")
            ParsedCommand = RF::Command::ABORT;
        else if (input_line == "ABORT_PAD")
            ParsedCommand = RF::Command::ABORT_PAD;
        else if (input_line == "ABORT_GROUND")
            ParsedCommand = RF::Command::ABORT_GROUND;
        else if (input_line == "TestTVC")
            ParsedCommand = RF::Command::TestTVC;
        else if (input_line == "ChirpTVC")
            ParsedCommand = RF::Command::ChirpTVC;
        else if (input_line == "Idle")
            ParsedCommand = RF::Command::Idle;
        else if (input_line == "Standby")
            ParsedCommand = RF::Command::Standby;
```


In Main.cpp, the while loop looks like:
```
while (mode.Update(navigation, controller, gps, igniter, imu, magnetometer, valveControl,  sparkPlug, pressureTransducer, loadCell, camera))
    {
    }
```

We either exit out of the while loop when the Mode is `Terminate` which returns false from `Update()` or if we hit `Abort`, before we ignite, then we exit the program right then and there

## Ground Control Communication
So, this is the Flight Software that runs on the Raspberry Pi but like I've mentioned in [Flow](#flow), the flight software is expecting to see commands through the radio which is where Ground Control comes in. All Ground Control is found in [Ground Control Github Link](https://github.com/UConn-Rocketry/ground-control). So to actually communicate with the rocket, we use ground control and the xbees