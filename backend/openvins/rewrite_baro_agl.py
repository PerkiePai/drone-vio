#!/usr/bin/env python3
"""Rewrite /baro_height in an Isaac bag to carry the negated TRUE AGL instead of
height-above-takeoff. Feeds OpenVINS's baro-depth-prior (fi_use_baro_depth_prior)
the correct camera-to-ground depth at altitude (over deep/non-flat terrain,
height-above-takeoff != ground clearance). Images/imu/gt are passed through
unchanged (no image re-encode). Run inside the openvins:noetic container.

Usage: rewrite_baro_agl.py <in.bag> <out.bag> <agl.csv>
"""
import csv
import sys

import numpy as np
import rosbag

in_bag, out_bag, agl_csv = sys.argv[1], sys.argv[2], sys.argv[3]

rows = list(csv.DictReader(open(agl_csv)))
ts = np.array([int(r["ts_ns"]) for r in rows]) / 1e9
agl = np.array([float(r["agl_m"]) for r in rows])
print(f"AGL: {len(rows)} rows, median {np.median(agl):.1f} m, range [{agl.min():.1f},{agl.max():.1f}]")

n_baro = n_other = 0
with rosbag.Bag(out_bag, "w") as out:
    for topic, msg, t in rosbag.Bag(in_bag).read_messages():
        if topic == "/baro_height":
            tsec = msg.header.stamp.to_sec()
            a = float(np.interp(tsec, ts, agl))   # AGL at this time (terrain clearance)
            msg.point.z = -a                       # OV world-z-down convention
            n_baro += 1
        else:
            n_other += 1
        out.write(topic, msg, t)
print(f"rewrote {n_baro} /baro_height msgs (=-AGL); passed through {n_other} others -> {out_bag}")
