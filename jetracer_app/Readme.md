# JetRacer App

This repository contains the `JetRacer App` for managing updates, organizing executables, and running the necessary services for the JetRacer on the Jetson Nano.

## Directory Structure

The folder structure ensures proper organization and functionality:

```
JetRacer_App/
├── bin/               # Stores executables to be run by systemd services
├── logs/              # Logs directory to track version updates
│   └── version_logs.txt
├── scripts/           # Contains scripts, including the update script
│   └── update_software.py
├── temp/              # Temporary directory for handling downloaded files
└── README.md          # This documentation file
```

## Features

- **Automatic Updates**:
  - The script checks for new releases from GitHub.
  - Downloads and updates the executables (`RaceCar` and `controller`) only if a new release is available.

- **Service Management**:
  - Stops existing services (`RaceCar.service` and `Controller.service`).
  - Moves the updated executables to the `bin` directory.
  - Restarts the services with the new executables.

- **Folder Creation**:
  - Automatically creates the required directories (`bin`, `logs`, `temp`) if they do not exist.

### Configure Services

Ensure the systemd services are set up correctly:

#### `RaceCar.service`
```ini
[Unit]
Description=RaceCar Service
After=network.target

[Service]
ExecStart=/home/team02/jetracer_app/bin/RaceCar
Restart=always
RestartSec=5
User=team02

[Install]
WantedBy=multi-user.target
```

#### `Controller.service`
```ini
[Unit]
Description=Controller Service
After=network.target

[Service]
ExecStart=/home/team02/jetracer_app/bin/controller
Restart=always
RestartSec=5
User=team02

[Install]
WantedBy=multi-user.target
```

Place these service files in `/etc/systemd/system/`:
```bash
sudo nano /etc/systemd/system/RaceCar.service
sudo nano /etc/systemd/system/Controller.service
```

Enable and start the services:
```bash
sudo systemctl daemon-reload
sudo systemctl enable RaceCar.service
sudo systemctl enable Controller.service
sudo systemctl start RaceCar.service
sudo systemctl start Controller.service
```

### Run the Update Script

Run the `update_software.py` script to check for updates and apply them:
```bash
cd ~/jetracer_app/scripts
python3 update_software.py
```

## How It Works

1. **Checks for Updates**: The script compares the latest release on GitHub with the version in `logs/version_logs.txt`.
2. **Downloads Updates**: If a new release is available, it downloads the release assets to `temp/`.
3. **Updates Executables**: Moves new executables to `bin/`, removes old ones, and sets appropriate permissions.
4. **Restarts Services**: Stops the running services, updates the executables, and restarts the services.

## To Do / Improvements

- Automatically create the `RaceCar.service` and `Controller.service` files if they don’t exist.
- Study and refine the app folder structure and update methodology to align with industry standards.
- Implement periodic automatic updates (currently requires manual execution).

## Logs

The update process logs the latest version in `logs/version_logs.txt`. Ensure this file exists or is created by the script.


