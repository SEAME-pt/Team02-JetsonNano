#!/bin/bash

projectDir=JetsonNano
UserName=team02
IpAddress=10.21.221.71
configDir=$projectDir/config/Zenoh
systemdDir=$projectDir/config/Systemd
PathBin=/opt/vehicle/bin
PathEtc=/opt/vehicle/etc/zenoh
Pass=seameteam2
architecture=$(uname -m)

check_ssh_connection() {
    local host=$1
    local user=$2
    echo "Checking connection to $host..."
    # Use nc (netcat) to test connection with 5 second timeout
    if nc -G 5 -z "$host" 22 >/dev/null 2>&1; then
        # Test SSH login
        if sshpass -p "$Pass" ssh -q -o ConnectTimeout=5 "$user@$host" exit; then
            return 0
        fi
    fi
    return 1
}

setup_services() {
    echo "Checking and setting up systemd services..."
    sshpass -p "$Pass" ssh -t "$UserName"@"$IpAddress" "
        for service in middleware.service vehiclesystem.service controller.service; do
            if ! systemctl list-unit-files | grep -q \$service; then
                echo \"Installing \$service...\"
                echo \"$Pass\" | sudo -S cp /opt/vehicle/etc/systemd/\$service /etc/systemd/system/
                echo \"$Pass\" | sudo -S systemctl enable \$service
            fi
        done
        echo \"$Pass\" | sudo -S systemctl daemon-reload
    "
}

# Build docker image with appropriate platform flag
echo "Building docker image to build app..."
if [ "$architecture" = "arm64" ] || [ "$architecture" = "aarch64" ]; then
    echo "Building for ARM64 architecture..."
    docker build -f ./JetsonNano/deploy/dockerfiles/DockerfileDeployJetson \
        --build-arg projectDir=/$projectDir \
        -t final-app .
else
    echo "Building for non-ARM64 architecture with platform emulation..."
    docker buildx build -f ./JetsonNano/deploy/dockerfiles/DockerfileDeployJetson \
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
docker cp tmpapp:/home/$projectDir/coverage ./coverage


if check_ssh_connection "$IpAddress" "$UserName"; then
    echo "Stopping services on JetsonNano..."
    sshpass -p "$Pass" ssh -t "$UserName"@"$IpAddress" "echo $Pass | sudo -S systemctl stop middleware.service vehiclesystem.service controller.service"
    
    echo "Creating directories if they don't exist..."
    sshpass -p "$Pass" ssh -t "$UserName"@"$IpAddress" "
        echo $Pass | sudo -S mkdir -p $PathBin $PathEtc /opt/vehicle/etc/systemd
        echo $Pass | sudo -S chown $UserName:$UserName $PathBin $PathEtc /opt/vehicle/etc/systemd
    "

    echo "Send binary to jetson over scp"
    sshpass -p "$Pass" scp VehicleSystem XboxController MiddleWare "$UserName"@"$IpAddress":"$PathBin"
    sshpass -p "$Pass" scp ./$configDir/VehicleSystemConfig.json ./$configDir/ControllerConfig.json ./$configDir/MiddleWareConfig.json "$UserName"@"$IpAddress":"$PathEtc"
    sshpass -p "$Pass" scp ./$systemdDir/*.service "$UserName"@"$IpAddress":"/opt/vehicle/etc/systemd/"

    echo "Setting correct permissions..."
    sshpass -p "$Pass" ssh -t "$UserName"@"$IpAddress" "echo $Pass | sudo -S chmod 755 $PathBin/* $PathEtc/*"
    
    setup_services

    echo "Restarting services on JetsonNano..."
    sshpass -p "$Pass" ssh -t "$UserName"@"$IpAddress" "echo $Pass | sudo -S systemctl start middleware.service vehiclesystem.service controller.service"
else
    echo "ERROR: Cannot connect to Jetson at $IpAddress"
fi