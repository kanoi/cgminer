#!/bin/sh
#
# This is used by windows-build-wsl.txt for cross compiling cgminer to windows
# It is designed specifically for Ubuntu under Windows Subsystem for Linux
# If it needs updating, raise an issue on https://githib.com/kanoi/cgminer
# You can of course type the commands yourself and modify them as needed
#
# Change this if you choose to install/build mxe somewhere else
ho="$HOME/mxe"
#
if [ "k$1" = "k" ] ; then
 echo "ERR: Missing parameter(s)"
 exit 1
fi
# install the packages required by mxe - may require sudo
if [ "$1" = "a" ] ; then
 shift
 apt install -y build-essential libtool autotools-dev automake pkg-config p7zip-full autopoint \
  bison flex libgdk-pixbuf2.0-dev gperf intltool libtool-bin lzip python ruby unzip "$@"
 exit $?
fi
# download mxe
if [ "$1" = "mxe" ] ; then
 git clone https://github.com/mxe/mxe.git
 exit $?
fi
# build mxe libraries required for cgminer ... this takes a long time ...
if [ "$1" = "b" ] ; then
 shift
 MXE_TARGETS='i686-w64-mingw32.static' make -j 4 libusb1 pthreads curl ncurses pdcurses "$@"
 exit $?
fi
# setup the cgminer build
if [ "$1" = "c" -o "$1" = "d" ] ; then
 dbg="-O2"
 if [ "k$1" = "kd" ] ; then
  dbg="-g -O1"
 fi
 shift
 PATH="$ho/usr/bin:$PATH" PKG_CONFIG_PATH="$ho/usr/i686-w64-mingw32.static/lib/pkgconfig/" \
  CFLAGS="-W -Wall $dbg -static -fcommon" ./configure --host=i686-w64-mingw32.static --enable-static --disable-shared "$@"
 exit $?
fi
# build cgminer
if [ "$1" = "make" ] ; then
 shift
 PATH="$ho/usr/bin:$PATH" PKG_CONFIG_PATH="$ho/usr/i686-w64-mingw32.static/lib/pkgconfig/" make "$@"
 exit $?
fi
