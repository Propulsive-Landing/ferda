#!/bin/bash
#Configure terminal settings to be raw
stty -f /dev/cu.usbserial-110 9600 raw -echo -ixon

cat /dev/cu.usbserial-110 &
sleep 2  # let GPS boot
printf '$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n' > /dev/cu.usbserial-110
echo Sent NMEA sentence command
sleep 2  # wait
printf '$PMTK251,38400*27\r\n' > /dev/cu.usbserial-110
echo Sent Baud Rate change command
sleep 2  # wait
stty -f /dev/cu.usbserial-110 38400 raw -echo -ixon
sleep 2
printf '$PMTK220,100*2F\r\n' > /dev/cu.usbserial-110
echo Sent Update Rate change command

pkill -f "cat /dev/cu.usbserial-110"
