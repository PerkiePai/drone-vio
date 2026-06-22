#!/usr/bin/env python3
"""
Convert Isaac Sim VIO dataset (CSV + PNG) to a ROS1 bag for OpenVINS.

Run inside the openvins:noetic Docker container via run_isaac_to_bag.sh,
or directly after `source /opt/ros/noetic/setup.bash`.

Usage:
  python3 isaac_to_bag.py <run_dir> [--out output.bag] [--mono] [--no-gt]

run_dir: directory containing imu.csv, baro.csv, poses.csv, frames.csv,
         images/, cam_calib.json, takeoff.json

Output ROS1 bag topics:
  /imu0              sensor_msgs/Imu              50 Hz  (FRD body, m/s², rad/s)
  /cam0/image_raw    sensor_msgs/Image            10 Hz  (mono8 by default)
  /baro_height       geometry_msgs/PointStamped   50 Hz  (z = -(alt - takeoff_alt))
  /gt_pose           geometry_msgs/PoseStamped    50 Hz  (ENU position, world-z quaternion)
"""

import argparse
import csv
import json
import os
import sys
from pathlib import Path

import cv2
import numpy as np
import rosbag
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Header


def ros_time(ts_ns: int):
    """Return a rospy.Time-compatible object from nanoseconds."""
    from rospy import Time
    return Time(nsecs=int(ts_ns))


def make_header(ts_ns: int, frame_id: str = "") -> Header:
    h = Header()
    h.stamp = ros_time(ts_ns)
    h.frame_id = frame_id
    return h


def load_csv(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir", help="Isaac Sim dataset directory")
    ap.add_argument("--out", default="", help="Output bag path (default: <run_dir>/isaac_sim.bag)")
    ap.add_argument("--mono", action="store_true", default=True,
                    help="Convert images to mono8 (default: true)")
    ap.add_argument("--rgb", action="store_true",
                    help="Keep images as rgb8 instead of mono8")
    ap.add_argument("--no-gt", action="store_true", help="Skip GT pose topic")
    args = ap.parse_args()

    run_dir = Path(args.run_dir)
    if not run_dir.is_dir():
        sys.exit(f"ERROR: not a directory: {run_dir}")

    out_path = Path(args.out) if args.out else run_dir / "isaac_sim.bag"
    use_mono = not args.rgb
    write_gt = not args.no_gt

    # Load calibration
    calib = json.loads((run_dir / "cam_calib.json").read_text())
    takeoff = json.loads((run_dir / "takeoff.json").read_text())

    # Baro: AMSL pressure altitude at takeoff (frame 1)
    baro_rows = load_csv(run_dir / "baro.csv")
    takeoff_baro_alt = float(baro_rows[0]["pressure_altitude_m"])
    print(f"  Takeoff baro alt: {takeoff_baro_alt:.3f} m AMSL")

    # IMU rows
    imu_rows = load_csv(run_dir / "imu.csv")

    # Poses (ENU takeoff-anchored position + per-frame attitude when available)
    pose_rows = load_csv(run_dir / "poses.csv")
    # Build pose lookup by frame number (same cadence as IMU)
    pose_by_frame = {int(r["frame"]): r for r in pose_rows}

    # Per-frame quaternion present when poses.csv has qx,qy,qz,qw columns
    # (written by vio-recorder-pai.py). Fall back to static takeoff attitude for
    # datasets recorded with the old vio_recorder.py (x,y,z only).
    pose_has_quat = bool(pose_rows and "qx" in pose_rows[0])
    tq = takeoff["attitude_xyzw"]  # [qx, qy, qz, qw] — fallback
    if pose_has_quat:
        print("  GT orientation: per-frame quaternion from poses.csv")
    else:
        print("  GT orientation: static takeoff snapshot (old recorder — upgrade to vio-recorder-pai.py)")

    # Image frame list
    img_rows = load_csv(run_dir / "frames.csv")
    # Build set of image frame numbers for fast lookup
    img_frame_set = {}
    for r in img_rows:
        frame = int(r["frame"])
        # Fix stale paths: ignore stored path, reconstruct from run_dir
        img_path = run_dir / "images" / "cam0" / f"f{frame:06d}.png"
        img_frame_set[frame] = (int(r["ts_ns"]), img_path)

    total_imgs = len(img_frame_set)
    total_imu = len(imu_rows)
    print(f"  IMU rows: {total_imu}  |  Image frames: {total_imgs}")
    print(f"  Output: {out_path}")

    if calib.get("extrinsic_is_constant") is False:
        print("  NOTE: extrinsic_is_constant=false (stabilized gimbal).")
        print("        OpenVINS will use the takeoff snapshot T_imu_cam.")
        print("        For best accuracy, re-record with DOWN_STABILIZE=False.")

    # All events sorted by timestamp
    # Each event: (ts_ns, kind, payload)
    events = []

    for row in imu_rows:
        ts = int(row["ts_ns"])
        frame = int(row["frame"])
        events.append((ts, "imu", row))
        if frame in img_frame_set:
            events.append((ts, "img", frame))
        # Baro co-located with IMU (same timestamp)

    # Sort by timestamp; "imu" before "img" on ties so IMU arrives first
    events.sort(key=lambda e: (e[0], 0 if e[1] == "imu" else 1))

    # Baro lookup by frame number (same ts as IMU)
    baro_by_frame = {int(r["frame"]): r for r in baro_rows}

    written_imu = 0
    written_img = 0
    written_baro = 0
    written_gt = 0
    img_errors = 0

    with rosbag.Bag(str(out_path), "w") as bag:
        for ts_ns, kind, payload in events:
            if kind == "imu":
                row = payload
                frame = int(row["frame"])

                # --- IMU message ---
                msg = Imu()
                msg.header = make_header(ts_ns, "imu")
                msg.angular_velocity.x = float(row["wx"])
                msg.angular_velocity.y = float(row["wy"])
                msg.angular_velocity.z = float(row["wz"])
                msg.linear_acceleration.x = float(row["ax"])
                msg.linear_acceleration.y = float(row["ay"])
                msg.linear_acceleration.z = float(row["az"])
                # Unknown covariances → -1 diagonal (ROS convention)
                msg.orientation_covariance[0] = -1.0
                msg.angular_velocity_covariance[0] = -1.0
                msg.linear_acceleration_covariance[0] = -1.0
                bag.write("/imu0", msg, ros_time(ts_ns))
                written_imu += 1

                # --- Baro message ---
                if frame in baro_by_frame:
                    br = baro_by_frame[frame]
                    alt_abs = float(br["pressure_altitude_m"])
                    alt_rel = alt_abs - takeoff_baro_alt  # relative to takeoff (m, up positive)
                    # OV world-z-down convention: negate
                    z_world = -alt_rel
                    bmsg = PointStamped()
                    bmsg.header = make_header(ts_ns, "world")
                    bmsg.point.z = z_world
                    bag.write("/baro_height", bmsg, ros_time(ts_ns))
                    written_baro += 1

                # --- GT pose message ---
                if write_gt and frame in pose_by_frame:
                    pr = pose_by_frame[frame]
                    pmsg = PoseStamped()
                    pmsg.header = make_header(ts_ns, "world")
                    pmsg.pose.position.x = float(pr["x"])
                    pmsg.pose.position.y = float(pr["y"])
                    pmsg.pose.position.z = float(pr["z"])
                    if pose_has_quat:
                        pmsg.pose.orientation.x = float(pr["qx"])
                        pmsg.pose.orientation.y = float(pr["qy"])
                        pmsg.pose.orientation.z = float(pr["qz"])
                        pmsg.pose.orientation.w = float(pr["qw"])
                    else:
                        pmsg.pose.orientation.x = tq[0]
                        pmsg.pose.orientation.y = tq[1]
                        pmsg.pose.orientation.z = tq[2]
                        pmsg.pose.orientation.w = tq[3]
                    bag.write("/gt_pose", pmsg, ros_time(ts_ns))
                    written_gt += 1

            elif kind == "img":
                frame = payload
                ts_img, img_path = img_frame_set[frame]

                if not img_path.exists():
                    if img_errors == 0:
                        print(f"  WARN: image not found: {img_path}")
                    img_errors += 1
                    continue

                bgr = cv2.imread(str(img_path))
                if bgr is None:
                    img_errors += 1
                    continue

                if use_mono:
                    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
                    img_msg = Image()
                    img_msg.header = make_header(ts_ns, "cam0")
                    img_msg.height, img_msg.width = gray.shape
                    img_msg.encoding = "mono8"
                    img_msg.step = img_msg.width
                    img_msg.data = gray.tobytes()
                else:
                    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                    img_msg = Image()
                    img_msg.header = make_header(ts_ns, "cam0")
                    img_msg.height, img_msg.width = rgb.shape[:2]
                    img_msg.encoding = "rgb8"
                    img_msg.step = img_msg.width * 3
                    img_msg.data = rgb.tobytes()

                bag.write("/cam0/image_raw", img_msg, ros_time(ts_ns))
                written_img += 1

                if written_img % 100 == 0:
                    pct = 100 * written_img / total_imgs
                    print(f"  {written_img}/{total_imgs} images ({pct:.0f}%)", flush=True)

    print(f"\nDone.")
    print(f"  IMU messages:   {written_imu}")
    print(f"  Image messages: {written_img}  (errors: {img_errors})")
    print(f"  Baro messages:  {written_baro}")
    print(f"  GT poses:       {written_gt}")
    print(f"  Bag: {out_path}")


if __name__ == "__main__":
    main()
