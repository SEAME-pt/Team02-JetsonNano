# System Monitor

This requires the Zenoh-Router active and runing to have access to the values.

## Overview

This project provides a system monitoring executable (`system_monitor`) designed to collect and publish key system metrics from an autonomous vehicle platform, specifically targeting NVIDIA Jetson devices. It leverages the Zenoh data platform for efficient and reliable data dissemination. The collected metrics include CPU load, CPU usage, memory usage, GPU utilization, and system temperature. This data is intended for real-time monitoring, analysis, and integration with data visualization tools like Grafana, backed by InfluxDB for storage.

## Architecture

The system comprises the following key components:

1.  **`system_monitor` Executable:** This C++ application is responsible for:
    * Collecting system metrics using standard Linux tools and specific methods for Jetson devices (e.g., `tegrastats` for GPU usage).
    * Establishing a connection with a Zenoh router.
    * Publishing the collected metrics as Zenoh resources under the `/Vehicle/1/SystemMonitor/` key space.

2.  **Zenoh Router:** A central data broker that facilitates the communication between different Zenoh participants. It is configured using `zenoh.json5`. In this setup, the router is responsible for:
    * Accepting connections from the `system_monitor` publisher.
    * Potentially routing and storing the published data.
    * Connecting to InfluxDB instances (both cloud and local) via the Storage Manager plugin to persist the collected metrics.

3.  **`monitor_config.json`:** This configuration file dictates how the `system_monitor` executable connects to the Zenoh network. It defines the communication mode (peer), the endpoints to connect to, and listening endpoints.

4.  **InfluxDB:** A time-series database used to store the system metrics published by the `system_monitor`. The Zenoh router, through its Storage Manager plugin, is configured to write data to specific InfluxDB instances and databases (`jetracer`).

5.  **Grafana (Implicit):** While not directly part of this repository, the documentation mentions Grafana as a visualization tool. It would typically be configured to query the InfluxDB database to create dashboards displaying the collected system metrics.

## Functionality

The `system_monitor` performs the following actions:

* **CPU Load:** Retrieves the system's average load over the last 1, 5, and 15 minutes from `/proc/loadavg`. The application currently publishes the 1-minute average.
* **CPU Usage:** Calculates the percentage of CPU time spent in non-idle states by reading `/proc/stat`.
* **Memory Usage:** Determines the percentage of RAM currently in use by querying system information using `sysinfo`.
* **Temperature:** Reads the system temperature from a thermal zone file, typically located at `/sys/devices/virtual/thermal/thermal_zone0/temp`.
* **GPU Usage (Jetson Specific):** Executes the `tegrastats` utility and parses its output to extract the GPU utilization percentage.

These metrics are collected periodically (every 5 seconds by default) and published via Zenoh with the following key expressions:

* `/Vehicle/1/SystemMonitor/cpuLoad`
* `/Vehicle/1/SystemMonitor/cpuUsage`
* `/Vehicle/1/SystemMonitor/memory`
* `/Vehicle/1/SystemMonitor/temperature`
* `/Vehicle/1/SystemMonitor/gpuUsage`

The Zenoh router, configured with the Storage Manager plugin, subscribes to these key expressions and writes the received data to the configured InfluxDB instances. This allows for historical data analysis and visualization.

## Deployment and Compilation Instructions

The provided `deploy` folder contains scripts for cross-compiling the `system_monitor` executable for the NVIDIA Jetson platform using Docker.

### Prerequisites

* Docker installed on your development machine.

### Cross-Compilation Steps

1.  **Navigate to the project root directory in your terminal.**

2.  **Build the Docker image for cross-compilation:**

    ```bash
    docker buildx build -f ./deploy/dockerfiles/DockerfileDeployJetson \
        --platform linux/arm64 --load \
        --build-arg projectDir=$(basename $(pwd)) \
        -t final-app .
    ```

    * `--platform linux/arm64`: Specifies the target architecture as ARM64, which is used by Jetson devices.
    * `--load`: Exports the build result to the local Docker images store.
    * `--build-arg projectDir=$(basename $(pwd))`: Passes the current project directory name as a build argument.
    * `-t final-app`: Tags the resulting Docker image as `final-app`.

3.  **(Optional) Remove any previous temporary container:**

    ```bash
    docker rm -f tmpapp
    ```

4.  **Create a new Docker container from the built image:**

    ```bash
    docker create --name tmpapp final-app
    ```

5.  **Copy the compiled executable (`system_monitor`) from the container back to your local machine:**

    ```bash
    docker cp tmpapp:/home/tools/system_monitor/system_monitor ./system_monitor
    ```

    The executable will be located in your project's root directory.

## Running the Application

To run the `system_monitor` on the Jetson device, ensure that the Zenoh router is running with the appropriate configuration (`zenoh.json5`) and that the `monitor_config.json` is provided to the executable.

### Prerequisites on the Jetson

* Zenoh router installed and configured.
* The `system_monitor` executable copied to the Jetson.
* The `monitor_config.json` file copied to the Jetson.

### Execution Steps on the Jetson

1.  **Start the Zenoh router using the `zenoh.json5` configuration:**

    ```bash
    zenohd -c zenoh.json5
    ```

    Ensure the router starts successfully and listens on the configured endpoints.

2.  **Run the `system_monitor` executable, providing the `monitor_config.json` file as a command-line argument:**

    ```bash
    ./system_monitor monitor_config.json
    ```

    The `system_monitor` will now connect to the Zenoh router and start publishing system metrics. You should see output in the terminal indicating the published data.

## Verification

To verify the system is working correctly:

1.  **Check the `system_monitor` output:** Ensure that the application is running without errors and is periodically printing the system metrics to the console.

2.  **Monitor Zenoh activity:** Use Zenoh tools (if available) to observe the published data under the `/Vehicle/1/SystemMonitor/**` key space.

3.  **Inspect InfluxDB:** Query your InfluxDB instance (both cloud and local) to confirm that the system metrics are being written to the `jetracer` database. You should see measurements corresponding to `cpuLoad`, `cpuUsage`, `memory`, `temperature`, and `gpuUsage`.

4.  **Visualize with Grafana:** If you have a Grafana dashboard configured, check if the newly ingested data is being displayed correctly.

5.  **Compare with `tegrastats`:** On the Jetson terminal, run `tegrastats` directly and compare its output with the GPU usage reported by the `system_monitor`. This helps ensure the accuracy of the GPU metric collection.

## Potential Improvements and Future Work

The `instructions.txt` file outlines some potential improvements:

* **Adding More Metrics:** Consider incorporating additional relevant system metrics such as disk I/O, network statistics, and individual core CPU usage.
* **Configuration Flexibility:** Allow for more configurable publishing intervals and key expressions through the configuration file.
* **Error Handling and Resilience:** Implement more robust error handling for system calls and Zenoh operations.
* **Unit Tests:** Add unit tests to ensure the reliability and correctness of the metric collection functions.
* **Logging:** Integrate a logging mechanism for better debugging and monitoring of the `system_monitor` application.
* **Dynamic Configuration:** Explore options for dynamically updating the configuration of the `system_monitor` without requiring a restart.

This documentation provides a comprehensive overview of the system monitor project, its architecture, deployment, and usage. By following these instructions, you can effectively collect and utilize valuable system metrics from your autonomous vehicle platform.