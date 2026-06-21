#!/usr/bin/env python3
# MARS-LVIG / Livox gotcha: /livox/imu linear_acceleration is in units of g,
# but OpenVINS (and ROS sensor_msgs/Imu spec) expect m/s^2. Rescale accel by g
# and republish to /livox/imu_si. Gyro is already rad/s (left untouched).
import rospy
from sensor_msgs.msg import Imu

G = 9.80665

def main():
    rospy.init_node("imu_g_to_si")
    pub = rospy.Publisher("/livox/imu_si", Imu, queue_size=2000)

    def cb(m):
        m.linear_acceleration.x *= G
        m.linear_acceleration.y *= G
        m.linear_acceleration.z *= G
        pub.publish(m)

    rospy.Subscriber("/livox/imu", Imu, cb, queue_size=2000)
    rospy.loginfo("imu_g_to_si: /livox/imu (g) -> /livox/imu_si (m/s^2)")
    rospy.spin()

if __name__ == "__main__":
    main()
