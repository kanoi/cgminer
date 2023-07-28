#!/bin/bash
# Path to the chroot environment
CHROOT_PATH=/home/$USER/buster-crossdev

# Target architecture triplet
TARGET_ARCH=arm-linux-gnueabihf

# Project to build
PROJECT=cgminer-hestiia-edition

# Path to the source code on the host machine
SOURCE_CODE_DIR=/home/$USER/${PROJECT}

# Function to set up chroot environment and build the project
function cross_compile_build {
    # Mount necessary directories
    sudo mount -t proc /proc $CHROOT_PATH/proc
    sudo mount -t sysfs /sys $CHROOT_PATH/sys
    sudo mount -o bind /dev $CHROOT_PATH/dev

    # Enter chroot environment
    sudo chroot $CHROOT_PATH /bin/bash -c "apt-get install build-essential autoconf automake libtool \
                                            pkg-config libcurl4-openssl-dev libudev-dev \
                                            libusb-1.0-0-dev libncurses5-dev zlib1g-dev git -y"
                                            
    sudo chroot $CHROOT_PATH /bin/bash -c "cd /tmp"
    sudo chroot $CHROOT_PATH /bin/bash -c "export CC=$TARGET_ARCH-gcc"
    sudo chroot $CHROOT_PATH /bin/bash -c "export CXX=$TARGET_ARCH-g++"

    # remove build_arm forlder if exists
    if [ -d "$SOURCE_CODE_DIR/build_arm_debug" ]; then
        rm -rf "$SOURCE_CODE_DIR/build_arm_debug"
    fi

    # Copy project code to chroot
    sudo cp -r $SOURCE_CODE_DIR $CHROOT_PATH/tmp

    
    # Build the project inside chroot
    sudo chroot $CHROOT_PATH /bin/bash -c "cd /tmp/$PROJECT/ && ./scripts/build_armhf_debug.sh"

    # Copy the built project to the host machine

    sudo cp -r $CHROOT_PATH/tmp/$PROJECT/build_arm_debug $SOURCE_CODE_DIR/build_arm_debug

    sudo chown -R $USER:$USER $SOURCE_CODE_DIR/build_arm_debug

    # Clean up
    sudo chroot $CHROOT_PATH /bin/bash -c "cd /tmp/ && rm -rf *"

    # Exit chroot
    sudo chroot $CHROOT_PATH /bin/bash -c "exit"

    # Unmount directories
    sudo umount $CHROOT_PATH/proc
    sudo umount $CHROOT_PATH/sys
    sudo umount $CHROOT_PATH/dev
}

cross_compile_build