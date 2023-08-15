#!/bin/bash
source ./env
#refresh the systems
sudo apt update && sudo apt upgrade -y

# Install debootstrap for creating a chroot environment
sudo apt install debootstrap git -y

# Get dependency for native CGminer build
sudo apt install build-essential autoconf automake libtool \
                pkg-config libcurl4-openssl-dev libudev-dev \
                libusb-1.0-0-dev libncurses5-dev zlib1g-dev -y

# Create a chroot environment based on debian buster 10.13
sudo debootstrap buster $CHROOT_PATH

# Add armhf architecture to the chroot environment
sudo chroot $CHROOT_PATH /bin/bash -c "dpkg --add-architecture armhf"
sudo chroot $CHROOT_PATH /bin/bash -c "apt-get update"

# Install cross compiler
sudo chroot $CHROOT_PATH /bin/bash -c "apt install gcc-arm-linux-gnueabihf crossbuild-essential-armhf -y"

# Install cgminer dependencies into the chroot environment
sudo chroot $CHROOT_PATH /bin/bash -c "apt-get install autoconf automake libtool \
                                            pkg-config libusb-1.0-0-dev libcurl4-openssl-dev:armhf libudev-dev:armhf \
                                            libusb-1.0-0-dev:armhf libncurses5-dev:armhf zlib1g-dev:armhf -y"


# Create a mount location for the code:
sudo chroot $CHROOT_PATH /bin/bash -c "mkdir /root/$PROJECT"

# Get the source code
git clone https://github.com/hestiia-engineering/cgminer-hestiia-edition.git $SOURCE_CODE_DIR

# Mount the source code to the chroot environment
sudo mount -o bind $SOURCE_CODE_DIR $CHROOT_PATH/root/$PROJECT






