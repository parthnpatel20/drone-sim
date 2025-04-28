#!/usr/bin/env python3
import csv
import statistics

sensor = []
act = []

with open('drone_log.csv', newline='') as f:
    reader = csv.DictReader(f)
    for row in reader:
        sensor.append(float(row['sensor_interval_ms']))
        act.append(float(row['act_exec_ms']))

# compute stats
s_min, s_max, s_mean = min(sensor), max(sensor), statistics.mean(sensor)
a_min, a_max, a_mean = min(act),    max(act),    statistics.mean(act)

# count deadline misses
s_misses = sum(1 for x in sensor if x > 50.5)
a_misses = sum(1 for x in act    if x > 10.0)

print(f"\nSensor Interval (ms):  min={s_min:.3f}, mean={s_mean:.3f}, max={s_max:.3f}")
print(f"Actuation Exec (ms):   min={a_min:.3f}, mean={a_mean:.3f}, max={a_max:.3f}")
print(f"\nMissed deadlines:")
print(f"  Sensor (>50 ms):   {s_misses}")
print(f"  Actuation (>10 ms): {a_misses}\n")
