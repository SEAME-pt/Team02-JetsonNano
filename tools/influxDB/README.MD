# Python Dependencies for InfluxDB Data Processing Script

This document outlines the required setup and Python packages to run the `processdata.py` script, which reads raw telemetry data from InfluxDB Cloud, processes it (cleaning and averaging), and writes the treated data back to InfluxDB with new measurement names.

---

## 📌 Project Overview

We are using **InfluxDB Cloud** as the time-series database to collect telemetry data streamed from **Zenoh** using a custom `system_monitor` and a **Zenoh router**. The raw data arrives in string format and is stored in a shared bucket.

This script is executed periodically via a **cron job**. It processes the raw measurements by:

- Converting values to `float` (from string),
- Resampling to 1-minute average intervals,
- Filling missing values with `0`,
- Writing the treated data to the same bucket under a new measurement prefix (e.g., `treated_memory`).

This processed data is used for **Grafana dashboards** for real-time telemetry and monitoring of JetRacer system performance.

---

## ✅ Required Dependencies

| Package      | Description                                                                  | Install Command              |
|--------------|------------------------------------------------------------------------------|------------------------------|
| `influxdb3`  | Official InfluxDB v3 client for Python. Used to query and write data.        | `pip install influxdb3`      |
| `pandas`     | Powerful data analysis library used to manipulate and resample time-series.  | `pip install pandas`         |

---

## 📦 Automatically Installed Dependencies

These are commonly installed along with the required packages:

- `urllib3`: HTTP client used for communication with the InfluxDB API.
- `certifi`: Provides SSL certificates for secure HTTP requests.
- `numpy`: Used by pandas for fast numerical operations.
- `python-dateutil`: Used by pandas for flexible time/date parsing.

---

## ⚠️ macOS SSL Warning

If you're using macOS, you may see this warning:
This warning can typically be ignored unless you're encountering connectivity or certificate issues.

---

## Future improvements

 - use telegraf for faster and less intensive processing of the raw data. (real real time).
 - add the remaining measurements to the dashboard of grafana with corresponding monitoring.
 - create a service / script that periodically treats the data on a time interval.
 - Data time consistency, generate logs with timestamps of the last update to make the script run on a bigger interval when there are missing time periods of treated telemetry.

---

 # Link to the Dashboard [CLICK HERE](https://luiscarvalho4.grafana.net/public-dashboards/6dbf7796c49e437cb2a795883b08701f).
