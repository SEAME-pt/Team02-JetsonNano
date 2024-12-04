#!/bin/bash

# Set variables
BUILD_DIR=build
EXECUTABLE=i2c
TARGET_USER=team02
TARGET_IP=10.21.221.71
TARGET_DIR=/home/team02

# Build the project
cmake --build $BUILD_DIR

# Transfer the executable to the target machine
scp $BUILD_DIR/$EXECUTABLE $TARGET_USER@$TARGET_IP:$TARGET_DIR

# Run the executable on the target machine
ssh $TARGET_USER@$TARGET_IP "cd $TARGET_DIR && ./$EXECUTABLE"