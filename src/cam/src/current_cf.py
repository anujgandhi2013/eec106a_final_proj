#!/usr/bin/env python
from __future__ import print_function
import numpy as np
import rospy
from kinematics import tf_to_rbt, rbt_to_tf
from threading import Lock
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

HELMET_TAG = 'ar_marker_0'

lkp, lkp_lock = None, Lock()
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

    tf = rbt_to_tf(np.dot(tf_to_rbt(helm_tf), tf_to_rbt(copter_tf)))
    

    delt = (tf.translation.x - helm_tf.translation.x, tf.translation.y - helm_tf.translation.y, tf.translation.z - helm_tf.translation.z)
    # print(delt)
    import kinematics as ks
    print(ks.rot_to_quaternion(ks.rotation_3d(np.array([0, 0, 1]), 3.14/2)))
    print(ks.rot_to_quaternion(ks.rot_to_quaternion(ks.rotation_3d(np.array([0, 0, 1]), 3.14/2))))
    print(tf_to_rbt(helm_tf))
    # print(tf_to_rbt(copter_tf))
    # print(tf)
    print()

def main():
    global pub

    rospy.init_node('current_cf')
    pub = publisher('copter_pos', TransformStamped)

    rospy.Subscriber('/tf', TFMessage, ar_callback)
    rospy.Subscriber('/vicon/helmet/helmet', TransformStamped, helm_callback)
    rospy.spin()

if __name__ == '__main__':
    main()