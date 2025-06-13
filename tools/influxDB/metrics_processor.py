from influxdb_client_3 import InfluxDBClient3, Point
import pandas as pd

# Connect to InfluxDB
client = InfluxDBClient3(
    "https://eu-central-1-1.aws.cloud2.influxdata.com",
    database="jetracer",
    token="rYtXXREgOrb0Kd5DSkA4b--qI9AC1gHvIGfNK90Ne0yGHsIDAYkvyxKzgxDLonwTVhzclF8ZZoVk7R9atXeHbQ=="
)

# Measurement pairs: source -> treated destination
measurements = {
    "Vehicle/1/SystemMonitor/memory": "Vehicle/1/SystemMonitor/treated_memory",
    "Vehicle/1/SystemMonitor/temperature": "Vehicle/1/SystemMonitor/treated_temperature",
    "Vehicle/1/SystemMonitor/gpuUsage": "Vehicle/1/SystemMonitor/treated_gpuUsage",
    "Vehicle/1/SystemMonitor/cpuUsage": "Vehicle/1/SystemMonitor/treated_cpuUsage",
    "Vehicle/1/SystemMonitor/cpuLoad": "Vehicle/1/SystemMonitor/treated_cpuLoad"
}

time_range = "1 hour"

for original, treated in measurements.items():
    query = f"""
    SELECT *
    FROM "{original}"
    WHERE time >= now() - interval '{time_range}'
    """
    table = client.query(query)
    df = table.to_pandas()

    if df.empty:
        continue

    # Convert and clean data
    df['time'] = pd.to_datetime(df['time'])
    df.set_index('time', inplace=True)
    df['value'] = pd.to_numeric(df['value'], errors='coerce')
    df = df.dropna(subset=['value'])

    # Resample and fill gaps with 0
    df_resampled = df['value'].resample('1min').mean().fillna(0)

    # Write to treated measurement
    for ts, value in df_resampled.items():
        point = Point(treated).time(ts).field("value", float(value))
        client.write(org="SEA:ME", record=point)

client.close()
