#!/bin/bash
#create simbolic link (MB, MCBL, MCBR, ARM, ADIS) to ttyUSB ports
sudo cp 99-usb-serial.rules /etc/udev/rules.d/99-usb-serial.rules
