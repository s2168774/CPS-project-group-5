#!/bin/bash
sudo apt-get update 
sudo apt-get upgrade 
mkdir ~/bluez 
cd ~/bluez 
wget http://www.kernel.org/pub/linux/bluetooth/bluez-5.55.tar.xz 
tar xvf bluez-5.55.tar.xz 
cd bluez-5.55/ 
sudo apt-get install -y libusb-dev libdbus-1-dev libglib2.0-dev libudev-dev libical-dev libreadline-dev 
./configure 
sudo make 
sudo make install
sudo systemctl daemon-reload 
sudo systemctl restart bluetooth
