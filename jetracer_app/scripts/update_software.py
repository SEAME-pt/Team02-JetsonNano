import requests
import os
import shutil
import subprocess

# this script works in the following way
# requires shutil lib and python3
# jetracer_app folder contains the scripts folder with the update_software.py in it
# jetracer_app/scripts/update_software.py
# this folder (jetracer_app) should be put in the jetracers jetson nano home directory
# once there just run the python3 update_software.py

# it will create the required folders for propper organization
# jetracer_app/bin       - executables to be ran on the executablename.service are stored here
# jetracer_app/logs      - log registry to ensure the update process happens only when a new release has come ou
# jetracer_app/temp      - temporary directory to handle the downloaded files before changng them to bin
# jetracer_app/scripts   - scripts location folders

# To Improve
# Does not create the RaceCar.service and controller.serice if they dont exist
# app folder structure and update/ runing methodology should be studied to comply and follow industry standards
# implemente automatic update intervals (currently only runs if user manualy triggers the script)

REPO_OWNER = "SEAME-pt"
REPO_NAME = "Team02-PyRacer"

EXECUTABLE_NAMES = ["RaceCar", "controller"]
SERVICE_NAMES = ["RaceCar.service", "Controller.service"] 
CURRENT_VERSION_FILE = "../logs/version_logs.txt"
TEMP_DIR = "../temp"               
BIN_DIR = "../bin"     

#fetch the latest release and compare with logs
def get_latest_release():
    """Fetch the latest release details from GitHub."""
    url = f"https://api.github.com/repos/{REPO_OWNER}/{REPO_NAME}/releases/latest"
    response = requests.get(url)
    response.raise_for_status()
    release_data = response.json()
    return release_data["tag_name"], release_data["assets"]

#downloads the release assets 
def download_asset(asset_url, output_path):
    """Download a release asset from GitHub."""
    response = requests.get(asset_url, stream=True, headers={"Accept": "application/octet-stream"})
    response.raise_for_status()
    with open(output_path, "wb") as f:
        for chunk in response.iter_content(chunk_size=8192):
            f.write(chunk)

def get_current_version():
    """Retrieve the current version stored locally. If it doesn't exist, create it."""
    if not os.path.exists(CURRENT_VERSION_FILE):
        # Create the file if it doesn't exist
        os.makedirs(os.path.dirname(CURRENT_VERSION_FILE), exist_ok=True)  # Ensure the directory exists
        with open(CURRENT_VERSION_FILE, "w") as f:
            f.write("")  # Initialize with an empty string
        return None  # No version yet

    # Read the existing version
    with open(CURRENT_VERSION_FILE, "r") as f:
        return f.read().strip()

def save_current_version(version):
    """Save the current version locally."""
    with open(CURRENT_VERSION_FILE, "w") as f:
        f.write(version)

def stop_services():
    """Stop systemd services."""
    for service in SERVICE_NAMES:
        try:
            subprocess.run(["sudo", "systemctl", "stop", service], check=True)
            print(f"Stopped service: {service}")
        except subprocess.CalledProcessError as e:
            print(f"Failed to stop service {service}: {e}")

def restart_services():
    """Restart and reload systemd services."""
    for service in SERVICE_NAMES:
        try:
            # Reload daemon and enable service
            subprocess.run(["sudo", "systemctl", "daemon-reload"], check=True)
            subprocess.run(["sudo", "systemctl", "enable", service], check=True)
            # Start the service
            subprocess.run(["sudo", "systemctl", "start", service], check=True)
            print(f"Started service: {service}")
        except subprocess.CalledProcessError as e:
            print(f"Failed to restart service {service}: {e}")

def move_executables():
    """Move executables from temp to bin, remove old ones, and set full permissions."""
    os.makedirs(BIN_DIR, exist_ok=True)  # Ensure the bin directory exists

    for executable in EXECUTABLE_NAMES:
        bin_path = os.path.join(BIN_DIR, executable)
        
        # Remove the old executable if it exists
        if os.path.exists(bin_path):
            os.remove(bin_path)
            print(f"Removed old executable: {executable} from bin.")

        # Move the new executable from temp to bin
        temp_path = os.path.join(TEMP_DIR, executable)
        if os.path.exists(temp_path):
            shutil.move(temp_path, bin_path)
            print(f"Moved {executable} from temp to bin.")
            
            # Set full permissions to the executable
            os.chmod(bin_path, 0o777)
            print(f"Set full permissions for {executable} in bin.")
        else:
            print(f"{executable} not found in temp.")

def main():
    try:
        # Get the current version
        current_version = get_current_version()
        print(f"Current version: {current_version}")

        # Fetch the latest release
        latest_version, assets = get_latest_release()
        print(f"Latest version: {latest_version}")

        if current_version == latest_version:
            print("The latest version is already installed. No update needed.")
            return

        # Download the new version
        print("Downloading new version...")
        os.makedirs(TEMP_DIR, exist_ok=True)  # Ensure temp directory exists
        for asset in assets:
            asset_url = asset["browser_download_url"]
            output_path = os.path.join(TEMP_DIR, asset["name"])
            download_asset(asset_url, output_path)
            print(f"Downloaded {asset['name']} to {output_path}")

        # Save the new version information
        save_current_version(latest_version)
        print(f"Updated version information to {latest_version}.")

        # Stop the systemd services
        stop_services()

        # Move executables from temp to bin
        move_executables()

        # Restart the systemd services
        restart_services()

    except Exception as e:
        print(f"An error occurred: {e}")


if __name__ == "__main__":
    main()
