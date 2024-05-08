#!/bin/bash
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install -y lzip build-essential git autoconf autopoint bison flex gperf libtool libtool-bin python ruby scons unzip intltool p7zip-full libgtk2.0-dev libssl-dev -y
sudo apt-get install -y lftp zip pv pixz
cd /opt
sudo git clone https://github.com/mxe/mxe.git
export MXE_DIR=/opt/mxe
export MXE_TARGETS='i686-w64-mingw32.static'
sudo make -j 4 -C $MXE_DIR MXE_TARGETS="$MXE_TARGETS" curl pthreads pdcurses ncurses libusb1 jansson libevent libmicrohttpd
echo 'export PATH=/opt/mxe/usr/bin:$PATH' >> ~/.profile
echo 'export PKG_CONFIG_PATH=/opt/mxe/usr/i686-w64-mingw32.static/lib/pkgconfig/' >> ~/.profile
export PATH=/opt/mxe/usr/bin:$PATH
export PKG_CONFIG_PATH=/opt/mxe/usr/i686-w64-mingw32.static/lib/pkgconfig/
cd ~
echo "environement ok"
echo "reboot ubuntu"

