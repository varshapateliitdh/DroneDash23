#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry

def odom_callback(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     rospy.Time.now(),
                     "base_link",
                     "odom")

if __name__ == '_main_':
    rospy.init_node('odom_base_link_transform')
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.spin()