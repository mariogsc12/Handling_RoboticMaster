from geometry_msgs.msg import Pose

def frame2pose(frame):
    rotation = frame.M
    qx, qy, qz, qw = rotation.GetQuaternion()
    pose = Pose()
    pose.position.x = frame.p.x()
    pose.position.y = frame.p.y()
    pose.position.z = frame.p.z()
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose

def get_joint_limits(ur10) -> list:
    min_limits = ur10.get_limites_inferiores()
    max_limits = ur10.get_limites_superiores()
    joint_limits = []
    for i in range(len(min_limits)):
        joint_limits.append((min_limits[i], max_limits[i]))

    return joint_limits
