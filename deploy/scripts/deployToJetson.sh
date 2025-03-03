#!/bin/bash

projectDir=JetsonNano
piUserName=team02
piIpAddress=10.21.221.71
piPath=/home/team02
piPass=seameteam2

echo "build docker image to build app"
docker buildx build --platform linux/arm64 --load -f ./JetsonNano/deploy/dockerfiles/DockerfileDeployJetson \
    --build-arg projectDir=/$projectDir \
    -t final-app .
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
