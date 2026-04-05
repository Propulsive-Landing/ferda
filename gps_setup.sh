#!/bin/bash
PORT="/dev/ttyUSB0"

echo "Activating virtual environment..."
source /home/pi/Documents/venv/bin/activate

echo "Waiting until GPS is fixed" 
python gps_startup.py 
echo "GPS is fix"

# Configure terminal settings to be raw
stty -F "$PORT" 9600 raw -echo -ixon
sleep 3  # let GPS boot

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

