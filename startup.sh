#!/bin/sh
GPSPORT="/dev/ttyUSB0"
RFPORT="/dev/ttyUSB01"

echo "Activating virtual environment..."
source /home/pi/Documents/venv/bin/activate

echo "Waiting until GPS is fixed" 
python gps_startup.py 
echo "GPS has fix"

# Configure terminal settings for both ports to be raw
sudo stty -F "$GPSPORT" 9600 raw -echo -ixon
sleep 3  # let GPS boot
sudo stty -F "$RFPORT" 9600 raw -echo -ixon

# Configure GPS Settings
printf '$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n' > "$PORT"
echo "Sent NMEA sentence command"
sleep 2  # wait

printf '$PMTK251,38400*27\r\n' > "$PORT"
echo "Sent Baud Rate change command"
sleep 2  # wait

stty -F "$PORT" 38400 raw -echo -ixon
sleep 2

printf '$PMTK220,100*2F\r\n' > "$PORT"
echo "Sent Update Rate change command"

cd build
sudo ./Ferda
