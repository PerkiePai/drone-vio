#!/usr/bin/env python3
"""Relay DJI local_position to /baro_height for baro-aided OpenVINS.

/dji_osdk_ros/local_position is geometry_msgs/PointStamped in ENU (z-up,
metres above takeoff home point).  OpenVINS world-z points DOWN (gravity +z).
We just negate z and republish -- no artificial noise because this is a real
sensor, not a synthetic signal derived from GT.

Usage:
  python3 relay_local_pos_as_baro.py  [_src_topic:=/dji_osdk_ros/local_position]
"""
import rospy
from geometry_msgs.msg import PointStamped

pub = None
first_z = None

def cb(msg: PointStamped):
    global pub, first_z
    # Negate: ENU z-up  →  world z-down (OpenVINS convention)
    out = PointStamped()
    out.header = msg.header
    out.point.z = -msg.point.z
    pub.publish(out)

rospy.init_node("baro_relay", anonymous=True)
pub = rospy.Publisher("/baro_height", PointStamped, queue_size=200)
src = rospy.get_param("~src_topic", "/dji_osdk_ros/local_position")
rospy.Subscriber(src, PointStamped, cb, queue_size=200)
rospy.loginfo(f"[baro_relay] {src} → /baro_height  (z negated for OV world-z-down)")
rospy.spin()
