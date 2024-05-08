#!/bin/bash
export folder=$(pwd)
cd ~
git clone https://github.com/wareck/cgminer-gekko.git
cd cgminer-gekko
autoreconf -fi
CFLAGS="-O2 -msse2" ./configure --host=i686-w64-mingw32.static --disable-shared --enable-gekko --enable-bflsc  --enable-bitforce --enable-bitfury --enable-cointerra --enable-drillbit --enable-hashfast --enable-hashratio --enable-icarus  --enable-klondike  --enable-modminer
make
strip cgminer.exe
cp cgminer.exe /tmp/
cd /tmp/
wget https://tinyurl.com/oqw7crn5 -O cgminer-gekko-win32.zip
unzip cgminer-gekko-win32.zip
mv cgminer.exe cgminer-gekko-win32
cd /tmp/
version=`git ls-remote -h https://github.com/wareck/cgminer-gekko.git | awk '{print $1}' |cut -c1-7`
7z a cgminer-gekko-$version.7z cgminer-gekko-win32
echo ""
echo "cgminer-gekko-$version.7z is ready in /tmp folder"
echo ""

