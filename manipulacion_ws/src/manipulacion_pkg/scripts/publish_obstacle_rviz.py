#!/usr/bin/env python
import rospy
import rospkg
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

def publish_marker():
    rospy.init_node('marker_publisher', anonymous=True)
    marker_pub = rospy.Publisher('obstacle_marker', Marker, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        marker = Marker()
        marker.header.frame_id = "base_link"  # Set the frame ID
        marker.header.stamp = rospy.Time.now()

        marker.ns = "obstacles"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = 0.45
        marker.pose.position.y = 0
        marker.pose.position.z = 0.15
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1

        marker.scale.x = 0.3
        marker.scale.y = 1.0
        marker.scale.z = 0.3

        marker.color.a = 1.0  # Alpha
        marker.color.r = 1.0  # Red
        marker.color.g = 0.0  # Green
        marker.color.b = 0.0  # Blue

        marker.lifetime = rospy.Duration()

        marker_pub.publish(marker)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_marker()
    except rospy.ROSInterruptException:
        pass
