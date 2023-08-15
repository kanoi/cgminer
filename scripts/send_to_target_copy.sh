#!/bin/bash

# Source and target paths
SOURCE_PATH="."
TARGET_PATH="debian@beaglelight:~/cgminer-hestiia-edition/"

# Rsync command
rsync -avz --progress --exclude='.git' "$SOURCE_PATH" "$TARGET_PATH"
