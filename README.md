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


## Hardware Config

  # Device Symbolic Linking
    1. Create a Shell Script
    First, create a shell script that will perform the device detection and symbolic link creation.
  
    Script: `/home/pi/link_iio_devices.sh`
  
    ```
    #!/bin/bash
  
    # Define the home directory for symbolic links
    HOME_DIR="/home/your_username"
    
    # Define the symbolic links
    IMU_LINK="${HOME_DIR}/imu_device"
    BAROMETER_LINK="${HOME_DIR}/barometer_device"
    GYROSCOPE_LINK="${HOME_DIR}/gyroscope_device"
    
    # Remove old links if they exist
    rm -f "$IMU_LINK" "$BAROMETER_LINK" "$GYROSCOPE_LINK"
    
    # Iterate over the IIO device directories
    for device in /sys/bus/iio/devices/iio:device*; do
        if [[ -d "$device" ]]; then
            if [[ -f "$device/in_accel_scale_available" ]]; then
                ln -s "$device" "$IMU_LINK"
                echo "IMU device found: $device"
            elif [[ -f "$device/in_temp_scale" ]]; then
                ln -s "$device" "$BAROMETER_LINK"
                echo "Barometer device found: $device"
            elif [[ -f "$device/in_anglvel_scale" ]]; then
                ln -s "$device" "$GYROSCOPE_LINK"
                echo "Gyroscope device found: $device"
            fi
        fi
    ```
  
    2. Make the Script Executable
    Run the following command to make the script executable:
  
    ```
    sudo chmod +x /usr/local/bin/link_iio_devices.sh
    ```
  
    3. Create the Systemd Service File
    Next, create a systemd service file to run this script at startup.
  
    Service File: /etc/systemd/system/link_iio_devices.service
  
    ```
    [Unit]
    Description=Link IIO Devices
    After=local-fs.target
    
    [Service]
    Type=oneshot
    ExecStart=/usr/local/bin/link_iio_devices.sh
    RemainAfterExit=yes
    
    [Install]
    WantedBy=multi-user.target
    ```
  
    4. Enable the Systemd Service
    Enable the service so that it runs at startup:
  
    ```
    sudo systemctl enable link_iio_devices.service
    ```
    
    5. Start the Service (Optional)
    You can start the service immediately (for testing purposes) using:
    
    ```
    sudo systemctl start link_iio_devices.service
    ```
  
    Home Directory: Replace `pi` in the script with the actual username or adjust the home directory path as needed.
    Permissions: Ensure that your user has permission to create symbolic links in the specified home directory.
    Testing: After creating the service, you can check the status with sudo systemctl status link_iio_devices.service to see if it executed successfully.
