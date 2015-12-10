#!/usr/bin/env python
import numpy as np
import rospy
from kinematics import VICON_TO_CRAZY, get_yaw_from_quat, rbt_to_pose, tf_to_rbt
from geometry_msgs.msg import TransformStamped, Pose, Point, Quaternion

pub = None
HELM_TO_DESIRED = np.array([[1, 0, 0, -.5], [0, 1, 0, 0], [0, 0, 1, -.1], [0, 0, 0, 1]])

def helm_callback(ts):
    helm_tf = ts.transform

    origin_to_helm = tf_to_rbt(helm_tf)
    origin_to_desired = np.dot(origin_to_helm, HELM_TO_DESIRED)
    origin_to_desired_crazy = np.dot(VICON_TO_CRAZY, origin_to_desired)

    desired_loc = rbt_to_pose(origin_to_desired_crazy)
    pub.publish(desired_loc)

def main():
    global pub

    rospy.init_node('desired_position')
    pub = rospy.Publisher('desired_loc', Pose, queue_size=10)

    rospy.Subscriber('/vicon/helmet/helmet', TransformStamped, helm_callback)
    rospy.spin()

if __name__ == '__main__':
    main()