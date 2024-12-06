#!/bin/bash

# Set variables
BUILD_DIR1=CarController/build
BUILD_DIR2=XboxController/build
EXECUTABLE1=RaceCar
EXECUTABLE2=controller
TARGET_USER=team02
TARGET_IP=10.21.221.71
TARGET_DIR=/home/team02

# Build the project

cmake --build $BUILD_DIR1

cmake --build $BUILD_DIR2

# Remove old executables on the target machine
ssh $TARGET_USER@$TARGET_IP "cd $TARGET_DIR && rm -rf ./$EXECUTABLE1 ./$EXECUTABLE2"

# Transfer the executables to the target machine
scp $BUILD_DIR1/$EXECUTABLE1 $TARGET_USER@$TARGET_IP:$TARGET_DIR
scp $BUILD_DIR2/$EXECUTABLE2 $TARGET_USER@$TARGET_IP:$TARGET_DIR

# Run the executables on the target machine in different processes
# ssh $TARGET_USER@$TARGET_IP "cd $TARGET_DIR && ./$EXECUTABLE1 && ./$EXECUTABLE2