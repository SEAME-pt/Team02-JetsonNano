#!/bin/bash

projectDir=JetsonNano
piUserName=team02
piIpAddress=10.21.221.71
piPath=/home/team02
piPass=seameteam2


architecture=$(uname -m)
echo "Detected architecture: $architecture"

# Build docker image with appropriate platform flag
echo "Building docker image to build app..."
if [ "$architecture" = "arm64" ] || [ "$architecture" = "aarch64" ]; then
    echo "Building for ARM64 architecture..."
    docker build -f ./JetsonNano/deploy/dockerfiles/DockerfileDeployRasp \
        --build-arg projectDir=/$projectDir \
        -t final-app .
else
    echo "Building for non-ARM64 architecture with platform emulation..."
    docker buildx build -f ./JetsonNano/deploy/dockerfiles/DockerfileDeployRasp \
        --platform linux/arm64 --load \
        --build-arg projectDir=/$projectDir \
        -t final-app .
fi

echo "Remove tmpapp container if it is exist"
docker rm -f tmpapp
echo "Create a tmp container to copy binary"
docker create --name tmpapp final-app
echo "Copy the binary from tmp container"
docker cp tmpapp:/home/$projectDir/VehicleSystem ./VehicleSystem
docker cp tmpapp:/home/$projectDir/XboxController ./XboxController
docker cp tmpapp:/home/$projectDir/MiddleWare ./MiddleWare
echo "Send binary to rasp over scp"
sshpass -p "$piPass" scp VehicleSystem XboxController MiddleWare "$piUserName"@"$piIpAddress":"$piPath"
