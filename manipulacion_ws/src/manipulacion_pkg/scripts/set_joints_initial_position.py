#!/usr/bin/env python3

import rospy
import numpy as np
from scipy.interpolate import CubicSpline
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

rospy.init_node('set_joints_initial_position', anonymous=True)

# Create a publisher on the command topic
pub = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)


# Wait for the publisher to establish a connection to subscribers
rospy.sleep(1)



trajectory_points = []


point = JointTrajectoryPoint()
point.positions = [0,-1,0,0,0,0]
point.time_from_start = rospy.Duration(0.1)
trajectory_points.append(point)

# Create the trajectory message
trajectory = JointTrajectory()
trajectory.header.stamp = rospy.Time.now()
trajectory.joint_names = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint', 
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint'
]
trajectory.points = trajectory_points

# Publish the trajectory
pub.publish(trajectory)

