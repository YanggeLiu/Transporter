#!/bin/sh

#Writen by Yangge 28.1.2020

echo '-------install-------- '

sudo apt-get -y update
sudo apt-get -y install build-essential cmake unzip pkg-config
sudo apt-get -y install libjpeg-dev libpng-dev libtiff-dev
sudo apt-get -y install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
sudo apt-get -y install libxvidcore-dev libx264-dev
sudo apt-get -y install libgtk-3-dev
sudo apt-get -y install libcanberra-gtk*
sudo apt-get -y install libatlas-base-dev gfortran
sudo apt-get -y install python3-dev

echo '-------unzip opencv-----'
sudo unzip -d /home/pi opencv-4.2.0.zip
sudo unzip -d /home/pi opencv_contrib-4.2.0.zip

echo '-------install pip------'
wget https://bootstrap.pypa.io/get-pip.py
sudo python3 get-pip.py
pip install numpy

echo '-------cmake opencv---'
cd /home/pi/opencv-4.2.0
sudo mkdir build
cd build

sudo cmake -D CMAKE_BUILD_TYPE=RELEASE \
	-D CMAKE_INSTALL_PREFIX=/usr/local \
	-D OPENCV_EXTRA_MODULES_PATH=/home/pi/opencv_contrib-4.2.0/modules \
	-D ENABLE_NEON=ON \
	-D ENABLE_VFPV3=ON \
	-D BUILD_TESTS=OFF \
	-D OPENCV_ENABLE_NONFREE=ON \
	-D INSTALL_PYTHON_EXAMPLES=OFF \
	-D CMAKE_SHARED_LINKER_FLAGS='-latomic' \
	-D BUILD_EXAMPLES=OFF ..

echo '-------increase the SWAP----'
sudo sed -i '16s/100/2048/g' /etc/dphys-swapfile
sudo /etc/init.d/dphys-swapfile stop
sudo /etc/init.d/dphys-swapfile start

echo '-------compile opencv-------'
make -j4

echo '-------install opencv-------'
sudo make install
sudo ldconfig
echo '----done-----'
