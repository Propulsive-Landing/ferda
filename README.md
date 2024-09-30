# Ferda :rocket:

Holds all flight software for the UConn Propulsive Landing team rockets. :smile:

All source code utilizes [Hungarian Notation](https://www.cse.iitk.ac.in/users/dsrkg/cs245/html/Guide.htm).

Create features in branches originating from the `dev` branch. When a feature is complete, make a pull request to merge it into `dev`.

## Table of Contents

1. [How to Run](#how-to-run)
2. [Building the Source Code](#building-the-source-code)
3. [Hardware Configuration](#hardware-configuration)
   - [Serial Port Setup](#serial-port-setup)
   - [Startup Script](#startup-script)
4. [Software-in-the-Loop Testing](#software-in-the-loop-testing)
   - [SIL Testing Using Windows + WSL2](#sil-testing-using-windows--wsl2)
   - [SIL Testing Using Windows](#sil-testing-using-windows)
5. [Systemd Service Setup](#systemd-service-setup)

## How to Run

1. Clone this repo.
2. Install the CMake Tools extension on VS Code.
3. Run `sudo apt install libeigen3-dev`.
4. Build the repo (ensure you're in either debug or release mode depending on your need).
5. Run the executable that gets created in the `build/` folder.

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

### Serial Port Setup

The Xbee module is a radio module which is used by our flight computer to send and receive data from the ground control, such as when we want to instruct the rocket to launch, or when we want to tell it to abort. To use the Xbee from our Raspberry Pi, we must configure it properly, here's how:

1. Xbee is currently (9/28/2024) configured to act as a terminal that only outputs values when there is a newline character.
2. Use `stty` to configure the device:
   ```bash
   stty -F /dev/ttyS0
   ```
   (`/dev/ttyS0` may change per device).
3. Configure settings:
   ```bash
   stty -F /dev/ttyS0 -settingToDisable settingToEnable
   ```
   Disable settings using a minus sign and enable settings without it.
4. The device configuration should resemble:
   ```
   speed 9600 baud; line = 0;
   -echo
   ```

### Startup Script

Our Custom PCB communicates with various sensors through different protocols which generate multiple device files which may change. To account for the device files changing, we have a script which creates symbolic links (shortcuts) to each of the devices and each link is in a predictable location which is then referenced from the flight software. Here's how to add that script:

1. Create a shell script to perform device detection and symbolic link creation:
   `/home/pi/fsw_startup.sh`

   ```bash
   #!/bin/bash

   # Define the home directory for symbolic links
   HOME_DIR="/home/pi"
   LOG_FILE="/home/pi/fsw_startup.log"

   # Log function to append messages to the log file
   log() {
       echo "$(date '+%Y-%m-%d %H:%M:%S') - $1" >> "$LOG_FILE"
   }

   # Define the symbolic links
   ACCEL_LINK="${HOME_DIR}/accel_device"
   BAROMETER_LINK="${HOME_DIR}/barometer_device"
   GYROSCOPE_LINK="${HOME_DIR}/gyroscope_device"

   # Remove old links if they exist
   rm -f "$ACCEL_LINK" "$BAROMETER_LINK" "$GYROSCOPE_LINK"

   log "Starting device linking script..."

   sleep 0.5

   # Iterate over the IIO device directories
   for device in /sys/bus/iio/devices/iio:device*; do
       if [[ -d "$device" ]]; then
           if [[ -f "$device/in_accel_scale_available" ]]; then
               ln -s "$device" "$ACCEL_LINK"
               echo "IMU device found: $device"
               log "IMU device found and linked: $device"
           elif [[ -f "$device/in_pressure_scale" ]]; then
               ln -s "$device" "$BAROMETER_LINK"
               echo "Barometer device found: $device"
               log "Barometer device found and linked: $device"
           elif [[ -f "$device/in_anglvel_scale" ]]; then
               ln -s "$device" "$GYROSCOPE_LINK"
               echo "Gyroscope device found: $device"
               log "Gyro device found and linked: $device"
           fi
       fi
   done

   sleep 2.0

   # Uncomment if you want fsw to start automatically on next boot
   # cd /home/pi/ferda/build
   # /home/pi/ferda/build/Ferda
   ```

2. Make the script executable:
   ```bash
   sudo chmod +x fsw_startup.sh
   ```

## Software-in-the-Loop Testing

Software-in-the-loop (SIL) testing allows you to connect your flight software to MATLAB's Simulink environment for real-time simulation. Follow the steps below to set up and run the SIL testing environment.

### SIL Testing Using Windows + WSL2

#### Install WSL2

To run your flight software in a Linux environment on Windows, you need to install WSL2 (Windows Subsystem for Linux). Follow these steps:

1. Open PowerShell as Administrator and run:
   ```bash
   wsl --install
   ```

2. Set up your preferred Linux distribution (e.g., Ubuntu) as your default WSL instance.

3. Ensure WSL2 is set as the default version:
   ```bash
   wsl --set-default-version 2
   ```

4. Once your distribution is set up, update your package list:
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

### SIL Testing Using Windows (NOT TESTED)

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

The flight software should be connected with the Simulink simulation, sending actuator commands and receiving simulated sensor data.

## Systemd Service Setup

For convenience, it is preferable to have the startup script run automatically. We can accomplish this by using systemd as demonstrated below.

1. Create the service file to run the startup script at boot:
   `/etc/systemd/system/fsw_startup.service`

   ```ini
   [Unit]
   Description=Link IIO Devices
   After=local-fs.target

   [Service]
   Type=oneshot
   ExecStart=/home/pi/fsw_startup.sh
   RemainAfterExit=yes

   [Install]
   WantedBy=multi-user.target
   ```

2. Enable the service:
   ```bash
   sudo systemctl enable fsw_startup.service
   ```

3. Start the service (optional):
   ```bash
   sudo systemctl start fsw_startup.service
   ```
