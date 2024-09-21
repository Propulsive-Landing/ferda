# ferda
Holds all flight software for the UConn Propulsive Landing team rockets. :smile:

All source code written utilizes [Hungarian Notation](https://www.cse.iitk.ac.in/users/dsrkg/cs245/html/Guide.htm)

Create features in branches created from dev branch. When feature is complete, make a pull request to bring feature into dev

## How to run
1. Clone this repo
2. Make sure to install the CMake Tools extension on VS Code
3. Run `sudo apt install libeigen3-dev`
4. Build the repo (make sure you're in debug or release mode depending on which one you need)
5. Run the executable that gets created in the `build/` folder


## Building the source code
1. First clone the repository. `git clone https://github.com/Propulsive-Landing/ferda.git` (link may have changed)
2. Enter into the new folder `cd ferda`
3. Generate the build files using cmake
 - For release use: `cmake -Bbuild -DCMAKE_BUILD_TYPE=Release .` (this uses the actual hardware sensors)
 - For debug use: `cmake -Bbuild -DCMAKE_BUILD_TYPE=Debug .`
4. Enter the newly generated `build` folder: `cd build`
5. Build the source `make all`
6. Complete! You should now have an executable inside the `build` folder which you can run with `sudo ./Ferda`



## Hardware Config

### Startup Script
  1. Create a Shell Script
  First, create a shell script that will perform the device detection and symbolic link creation.
  
  Script: `/home/pi/fsw_startup.sh`

  ```
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
  
  
  # Uncomment if you want fsw to start automatically on next book
  # /home/pi/ferda/build/Ferda

  ```

  2. Make the Script Executable
  Run the following command to make the script executable:

  ```
  sudo chmod +x /usr/local/bin/fsw_startup.sh
  ```

  3. Create the Systemd Service File
  Next, create a systemd service file to run this script at startup.

  Service File: /etc/systemd/system/fsw_startup.service

  ```
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

  4. Enable the Systemd Service
  Enable the service so that it runs at startup:

  ```
  sudo systemctl enable fsw_startup.service
  ```
  
  5. Start the Service (Optional)
  You can start the service immediately (for testing purposes) using:
  
  ```
  sudo systemctl start fsw_startup.service
  ```

  Home Directory: Replace `pi` in the script with the actual username or adjust the home directory path as needed.
  Permissions: Ensure that your user has permission to create symbolic links in the specified home directory.
  Testing: After creating the service, you can check the status with sudo systemctl status link_iio_devices.service to see if it executed successfully.
