#!/bin/bash

# Configuration
TARGET_HOST="debian@beaglebone"
TARGET_DIR="/home/debian/"

# Check for release or debug argument
if [ "$1" = "release" ]; then
    BINARY_PATH="./build_arm/cgminer"
elif [ "$1" = "debug" ]; then
    BINARY_PATH="./build_arm_debug/cgminer"
else
    echo "Usage: $0 [release/debug]"
    exit 1
fi

# Check if the binary file exists
if [ ! -f "$BINARY_PATH" ]; then
    echo "Error: Binary not found: $BINARY_PATH"
    exit 1
fi

# Copy the binary to the target host using scp
scp "$BINARY_PATH" "$TARGET_HOST:$TARGET_DIR/$1"
