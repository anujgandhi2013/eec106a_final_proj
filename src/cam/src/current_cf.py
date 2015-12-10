#!/usr/bin/env python
import numpy as np
import rospy
from kinematics import tf_to_rbt, rbt_to_pose, VICON_TO_CRAZY
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, Pose, Point

HELMET_TAG = 'ar_marker_0'
HELM_TO_HELMAR = np.array([[0.03, 0.19, -0.98, -0.1], [-1, 0.05, -0.02, -0.01], [0.05, 0.98, 0.19, -0.09], [0, 0, 0, 1]])

lkp = None
pub = None

def ar_callback(msg):
    global lkp
    for trans in msg.transforms:
        if trans.child_frame_id == HELMET_TAG:
            lkp = trans.transform
            break

def helm_callback(ts):
    if not lkp:
        return

    helm_tf = ts.transform
    copter_tf = lkp
    copter_to_helmar = tf_to_rbt(copter_tf)

    origin_to_helm = tf_to_rbt(helm_tf)
    helmar_to_copter = np.linalg.inv(copter_to_helmar)

    origin_to_helmar = np.dot(origin_to_helm, HELM_TO_HELMAR)
    origin_to_copter = np.dot(origin_to_helmar, helmar_to_copter)
    origin_to_copter_crazy = np.dot(VICON_TO_CRAZY, origin_to_copter)

    copter_loc = rbt_to_pose(origin_to_copter_crazy)
    pub.publish(copter_loc)

def main():
    global pub

    rospy.init_node('current_cf_raw')
    pub = rospy.Publisher('copter_pos_raw', Pose, queue_size=10)

    rospy.Subscriber('/tf', TFMessage, ar_callback)
    rospy.Subscriber('/vicon/helmet/helmet', TransformStamped, helm_callback)
    rospy.spin()

if __name__ == '__main__':
    main()