#!/bin/bash
#create simbolic link (MB, MCBL, MCBR, ARM) to ttyUSB port  
sudo cp 99-usb-serial.rules /etc/udev/rules.d/99-usb-serial.rules
