#!/usr/bin/env python3
"""
Synthetic barometer node for MARS-LVIG baro-aided VIO experiments.

Subscribes to a ground-truth NavSatFix topic from the bag, extracts altitude,
and republishes as geometry_msgs/PointStamped on /baro_height with:
  .point.z = -(altitude - altitude_ref) + gaussian_noise

Sign convention: OpenVINS world frame has z pointing DOWN (gravity = [0,0,+9.81]).
Altitude increases UP, so rising altitude → negative world-z → we negate.
altitude_ref = altitude at the FIRST message received (bag start), so the value
starts at 0 and matches VIO's initial p_z = 0.

Usage (inside Docker container, with roscore + bag playing):
  python3 /scripts/synthetic_baro.py \
      _gt_topic:=/gnss/fix \
      _noise_std:=0.5

Parameters (ROS private):
  ~gt_topic   : NavSatFix topic with RTK altitude (default /gnss/fix)
  ~noise_std  : 1-sigma Gaussian noise in metres (default 0.5)
  ~out_topic  : Output PointStamped topic (default /baro_height)

To find the correct gt_topic, run:
  backend/openvins/inspect_bag.sh
and look for sensor_msgs/NavSatFix topics.
"""
import rospy
import numpy as np
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped


class SyntheticBaro:
    def __init__(self):
        gt_topic = rospy.get_param("~gt_topic", "/gnss/fix")
        self.noise_std = float(rospy.get_param("~noise_std", 0.5))
        out_topic = rospy.get_param("~out_topic", "/baro_height")

        self.alt_ref = None
        self.pub = rospy.Publisher(out_topic, PointStamped, queue_size=200)
        rospy.Subscriber(gt_topic, NavSatFix, self._cb, queue_size=200)
        rospy.loginfo(f"[synthetic_baro] {gt_topic} -> {out_topic}  noise={self.noise_std:.2f} m")

    def _cb(self, msg: NavSatFix):
        alt = msg.altitude
        if np.isnan(alt) or np.isinf(alt):
            return

        # Initialise reference at first valid reading
        if self.alt_ref is None:
            self.alt_ref = alt
            rospy.loginfo(f"[synthetic_baro] reference altitude = {alt:.2f} m (WGS84 ellipsoidal)")

        # Convert to world-z (DOWN positive): negate altitude change
        z_world = -(alt - self.alt_ref) + np.random.normal(0.0, self.noise_std)

        ps = PointStamped()
        ps.header = msg.header
        ps.point.x = 0.0
        ps.point.y = 0.0
        ps.point.z = z_world
        self.pub.publish(ps)


if __name__ == "__main__":
    rospy.init_node("synthetic_baro", anonymous=True)
    SyntheticBaro()
    rospy.spin()
