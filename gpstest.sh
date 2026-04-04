#!/bin/bash

stty -f /dev/cu.usbserial-110 raw -echo -ixon
sleep 2  # let GPS boot