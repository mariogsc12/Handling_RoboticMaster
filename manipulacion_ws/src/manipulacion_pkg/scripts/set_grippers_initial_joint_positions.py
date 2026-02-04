#!/usr/bin/env python3
import rospy, rospkg
import yaml
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def set_initial_joint_positions(gripper_config_file):
    rospy.init_node('set_joint_position')

    # Load the configuration file
    with open(gripper_config_file, 'r') as file:
        config = yaml.load(file, Loader=yaml.FullLoader)
    
    joint_positions = config['joint_positions_open']
    topic_pub = config['topic_pub']
    pub = rospy.Publisher(topic_pub, JointTrajectory, queue_size=10)

    # Wait for the connection to establish by checking for subscribers
    rospy.loginfo("Waiting for a subscriber on %s...", topic_pub)
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
      rospy.sleep(0.5)
    rospy.loginfo("Subscriber detected on %s, proceeding to publish initial joint positions.", topic_pub)

    trajectory = JointTrajectory()
    trajectory.header.stamp = rospy.Time.now()
    trajectory.joint_names = list(joint_positions.keys())
    point = JointTrajectoryPoint()
    point.positions = list(joint_positions.values())
    point.time_from_start = rospy.Duration(2)
    trajectory.points = [point]

    pub.publish(trajectory)
    pub.publish(trajectory)

    rospy.sleep(2)
    rospy.loginfo("Published initial joint positions to %s", topic_pub)

if __name__ == '__main__':
    gripper_name = rospy.get_param('tipo_gripper')
    rospack = rospkg.RosPack()
    gripper_config_file = package_path = rospack.get_path('manipulacion_pkg')+'/config/grippers/'+gripper_name+'_hand_config.yaml'
    try:
        set_initial_joint_positions(gripper_config_file)
    except rospy.ROSInterruptException:
        pass
