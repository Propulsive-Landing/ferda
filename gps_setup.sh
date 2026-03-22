#!/bin/bash

stty -F /dev/ttyUSB0 9600 raw -echo -ixon
sleep 2  # let GPS boot
printf '$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n' > /dev/ttyUSB0
sleep 2  # wait
printf '$PMTK251,38400*27\r\n' > /dev/ttyUSB0
sleep 1  # wait
stty -F /dev/ttyUSB0 38400 raw -echo -ixon
sleep 2
printf '$PMTK220,100*2F\r\n' > /dev/ttyUSB0
