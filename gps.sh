#!/bin/bash


PORT="/dev/cu.usbserial-110"

stty -f "$PORT" 9600 raw -echo -ixon
sleep 2

# Open port once
exec 3<> "$PORT"

# Send command
printf '$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n' >&3
echo "Sent command"

# Read response for a few seconds
# timeout 5 cat <&3

# Close
exec 3>&-