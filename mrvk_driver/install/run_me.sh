#!/bin/bash
#create teensy rule
sudo cp 49-teensy.rules /etc/udev/rules.d/49-teensy.rules
#create simbolic link (MB, MCBL, MCBR) to ttyUSB ports
sudo cp 96-mrvk-boards.rules /etc/udev/rules.d/96-mrvk-boards.rules
#create simbolic link (ADIS16350 ADIS16488) to ttyUSB ports
sudo cp 97-gps.rules /etc/udev/rules.d/97-gps.rules
#create simbolic link (GPSS) to ttyUSB ports
sudo cp 98-adis-devices.rules /etc/udev/rules.d/98-adis-devices.rules
#create simbolic link (HOKUYO) to ttyUSB ports
sudo cp 99-hokuyo.rules /etc/udev/rules.d/99-hokuyo.rules
#add user to dailout
sudo adduser `echo $USER` dialout
