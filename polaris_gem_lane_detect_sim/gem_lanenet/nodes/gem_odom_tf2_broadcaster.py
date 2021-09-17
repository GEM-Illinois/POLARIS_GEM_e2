#!/usr/bin/env python3
import sys

import rospy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros


class OdomTFBroadcaster:
    def __init__(self, robot_name: str) -> None:
        self._robot_name = robot_name
        self._br = tf2_ros.TransformBroadcaster()
        self._prev_stamp = rospy.Time()

    def odom_callback(self, odom_msg: Odometry):
        if rospy.Time.now() <= self._prev_stamp:  # Avoid redundant update
            return  # Do nothing

        # NOTE this works with only one robot with respect to the world frame.
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = odom_msg.pose.pose.position.z
        t.transform.rotation.x = odom_msg.pose.pose.orientation.x
        t.transform.rotation.y = odom_msg.pose.pose.orientation.y
        t.transform.rotation.z = odom_msg.pose.pose.orientation.z
        t.transform.rotation.w = odom_msg.pose.pose.orientation.w
        self._br.sendTransform(t)

        self._prev_stamp = t.header.stamp


def main(argv):
    rospy.init_node('gem_odom_tf2_broadcaster')
    robot_name = rospy.get_param("~robot_name", "gem")

    bcast = OdomTFBroadcaster(robot_name)

    rospy.Subscriber("base_footprint/odom", Odometry,
                     bcast.odom_callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down Odom to TF broadcast node")


if __name__ == '__main__':
    sys.exit(main(rospy.myargv(sys.argv)) or 0)
