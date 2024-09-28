#!/bin/sh

sudo stty -F /dev/ttyS0 9600 -echo

cd build
sudo ./Ferda
