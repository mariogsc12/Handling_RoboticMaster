#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF

def get_joint_names_from_urdf():
    robot_description = rospy.get_param('/robot_description')
    robot = URDF.from_xml_string(robot_description)
    return [joint.name for joint in robot.joints if joint.type != 'fixed']

def set_joint_states(pub_joint_states, joint_positions):
    joint_names = get_joint_names_from_urdf()
    if len(joint_names) != len(joint_positions):
        raise ValueError('joint_names and joint_positions.name must be equal')
    jointState = JointState()
    jointState.header.stamp = rospy.Time.now()
    jointState.name = joint_names
    jointState.position = joint_positions
    pub_joint_states.publish(jointState)

rospy.init_node('set_initial_joint_position_node')
rospy.sleep(1.0)
pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
set_joint_states(pub, [0.0, -1.0, 0.0, 0.0, 0.0, 0.0])
rospy.sleep(1.0)
set_joint_states(pub, [0.0, -1.0, 0.0, 0.0, 0.0, 0.0])
rospy.loginfo('Done')