#!/usr/bin/env python3
# Scan a MARS-LVIG bag's /livox/imu for well-excited windows (turns/climbs) where
# OpenVINS dynamic init is observable. Prints top windows by accel+gyro variance.
# Run inside the openvins:noetic container.
import rosbag, numpy as np, sys
bag_path = sys.argv[1] if len(sys.argv) > 1 else '/data/mars-lvig/AMvalley01.bag'
WIN = 1.0  # seconds per window

ts, acc, gyr = [], [], []
b = rosbag.Bag(bag_path, 'r')
for i, (_, m, _) in enumerate(b.read_messages(topics=['/livox/imu'])):
    ts.append(m.header.stamp.to_sec())
    acc.append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
    gyr.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
    if i % 50000 == 0:
        print(f"  read {i} imu msgs...", flush=True)
b.close()
ts = np.array(ts); acc = np.array(acc); gyr = np.array(gyr)
t0 = ts[0]; rel = ts - t0
amag = np.linalg.norm(acc, axis=1); gmag = np.linalg.norm(gyr, axis=1)
print(f"\n/livox/imu: {len(ts)} msgs, {rel[-1]:.1f}s, mean|a|={amag.mean():.3f} mean|w|={gmag.mean():.3f}")

rows = []
t = 0.0
while t < rel[-1]:
    s = (rel >= t) & (rel < t + WIN)
    if s.sum() > 20:
        rows.append((t, amag[s].std(), gmag[s].std()))
    t += WIN
rows = np.array(rows)
# rank by combined excitation (gyro weighted: turns help most for observability)
score = rows[:, 1] + 5.0 * rows[:, 2]
order = np.argsort(-score)
print("\nTop 15 excited windows (t_rel  accel_std  gyro_std  score):")
for k in order[:15]:
    print(f"  t={rows[k,0]:7.1f}s  a_std={rows[k,1]:.3f}  g_std={rows[k,2]:.4f}  score={score[k]:.3f}")
print(f"\nLow-excitation baseline: median a_std={np.median(rows[:,1]):.3f} g_std={np.median(rows[:,2]):.4f}")
