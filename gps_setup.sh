#!/bin/bash

stty -F /dev/ttyUSB0 9600 raw -echo -ixon
sleep 2  # let GPS boot
printf '$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n' > /dev/ttyUSB0
