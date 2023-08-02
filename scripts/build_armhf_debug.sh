#!/bin/bash
mkdir -p ./build_arm_debug && cd ./build_arm_debug
../autogen.sh
CFLAGS="-O1 -ggdb -Wall -marm -fcommon" ../configure --enable-debug --build=x86_64-linux-gnu --host=arm-linux-gnueabihf --enable-bm1397
make