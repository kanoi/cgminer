mkdir -p ./build_x86_64 && cd build_x86_64
../autogen.sh
CFLAGS="-O2 -Wall -march=native -fcommon" ../configure --enable-bm1397
make