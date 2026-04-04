#!/bin/bash

# Create a variable to hold port name
PORT = "/dev/cu.usbserial-110" 

echo "using port $PORT"
# Configure terminal settings to be raw
# stty -f /dev/cu.usbserial-110 9600 raw -echo -ixon
# sleep 2  # let GPS boot

# # Open port for reading and writing 
# exex 3<> "$PORT"

# printf '$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n' > &3
# echo Sent Command
# sleep 2  # wait
# printf '$PMTK251,38400*27\r\n' > /dev/cu.usbserial-110
# sleep 1  # wait
# stty -f /dev/cu.usbserial-110 38400 raw -echo -ixon
# sleep 2
# printf '$PMTK220,100*2F\r\n' > /dev/cu.usbserial-110
