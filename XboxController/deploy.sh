#!/bin/bash

# Set variables
BUILD_DIR=build
EXECUTABLE=controller
TARGET_USER=team02
TARGET_IP=10.21.221.71
TARGET_DIR=/home/team02

# Build the project
mkdir $BUILD_DIR

cd $BUILD_DIR

cmake ..

cd ..

cmake --build $BUILD_DIR


ssh $TARGET_USER@$TARGET_IP "cd $TARGET_DIR && rm -rf ./$EXECUTABLE"

# Transfer the executable to the target machine
scp $BUILD_DIR/$EXECUTABLE $TARGET_USER@$TARGET_IP:$TARGET_DIR

# Run the executable on the target machine
# ssh $TARGET_USER@$TARGET_IP "cd $TARGET_DIR && ./$EXECUTABLE"