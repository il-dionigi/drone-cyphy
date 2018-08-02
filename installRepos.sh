#!/bin/bash

mkdir ../cyphy
cd ../cyphy

git clone https://dionigi@bitbucket.org/dionigi/crazyflie-firmware-cyphy.git

git clone https://dionigi@bitbucket.org/dionigi/crazyflie2-nrf-firmware-cyphy.git

git clone https://dionigi@bitbucket.org/dionigi/crazyflie-lib-python-cyphy.git
pip install -e ./crazyflie-lib-python-cyphy/

git clone https://dionigi@bitbucket.org/dionigi/lps-firmware-cyphy.git

mkdir ../crazyflie
cd ../crazyflie

git clone --recursive https://github.com/bitcraze/crazyflie-firmware.git
sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
sudo apt-get update
sudo apt-get install libnewlib-arm-none-eabi

git clone --recursive https://github.com/bitcraze/crazyflie2-nrf-firmware.git
sudo apt-get install gcc-arm-none-eabi gdb-arm-none-eabi binutils-arm-none-eabi

git clone --recursive https://github.com/bitcraze/crazyflie2-nrf-bootloader.git
./crazyflie2-nrf-bootloader/tools/build/download_deps.sh

git clone --recursive https://github.com/bitcraze/lps-tools.git
pip3 install -e ./lps-tools/[pyqt5]

git clone https://github.com/bitcraze/crazyflie-clients-python.git
sudo apt-get install python3 python3-pip python3-pyqt5 python3-pyqt5.qtsvg
pip3 install -e ./crazyflie-clients-python/

git clone --recursive https://github.com/bitcraze/lps-node-firmware.git
sudo apt-get install libncurses5:i386

git clone https://github.com/bitcraze/crazyflie-lib-python.git
sudo groupadd plugdev
sudo usermod -a -G plugdev dionigi
sudo touch /etc/udev/rules.d/99-crazyradio.rules
echo "SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"1915\", ATTRS{idProduct}==\"7777\", MODE=\"0664\", GROUP=\"plugdev\" " > tmpf
echo "SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"0483\", ATTRS{idProduct}==\"5740\", MODE=\"0664\", GROUP=\"plugdev\" " >> tmpf
cat tmpf >> /etc/udev/rules.d/99-crazyradio.rules
rm tmpf

git clone https://github.com/bitcraze/crazyradio-firmware.git
sudo apt-get install sdcc binutils