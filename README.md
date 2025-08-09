# remote-dispfmw
esp32 firmware for remote display with control and command over I2C interface

# Installation and build instructions
```bash 
sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0 libi2c-dev
mkdir -p ~/esp
cd ~/esp
git clone -b v5.5 --recursive https://github.com/espressif/esp-idf.git
cd ~/esp/esp-idf
./install.sh esp32c6
sudo usermod -a -G dialout $USER
exit and relogin
cd ~
git clone https://github.com/hackboxguy/remote-dispfmw.git
cd remote-dispfmw
. $HOME/esp/esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```
