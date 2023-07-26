#!/bin/bash
mkdir -p ./build_arm && cd ./build_arm
../autogen.sh
CFLAGS="-O2 -Wall -marm -fcommon" ../configure --build=x86_64-linux-gnu --host=arm-linux-gnueabihf --enable-bm1397 --disable-libcurl
make