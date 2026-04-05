#!/bin/bash

PORT = "/dev/cu.usbserial-110"

# Configure terminal settings to be raw
stty -f "$PORT" 9600 raw -echo -ixon

# In the background use cat to open up the termina
cat /dev/cu.usbserial-110 &
sleep 2  # let GPS boot

printf '$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n' > "$PORT"
echo Sent NMEA sentence command
sleep 2  # wait

printf '$PMTK251,38400*27\r\n' > "$PORT"
echo Sent Baud Rate change command
sleep 2  # wait

stty -f "$PORT" 38400 raw -echo -ixon
sleep 2

printf '$PMTK220,100*2F\r\n' > "$PORT"
echo Sent Update Rate change command

pkill -f "cat "$PORT"
