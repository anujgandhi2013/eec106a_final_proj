#!/usr/bin/env python
import numpy as np
import rospy
from kinematics import tf_to_rbt, rbt_to_pose, VICON_TO_CRAZY
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, Pose, Point

np.set_printoptions(precision=2)

HEL_TAG = 'ar_marker_1'
AR_TAG = 'ar_marker_0'
hel, ar = None, None
last_100 = []
def ar_callback(msg):
    global hel, ar, last_100
    for trans in msg.transforms:
        if trans.child_frame_id == HEL_TAG:
            hel = trans.transform
        elif trans.child_frame_id == AR_TAG:
            ar = trans.transform
    if hel is not None and ar is not None:
        hel_m = tf_to_rbt(hel)
        ar_m = tf_to_rbt(ar)
        trans = np.dot(np.linalg.inv(hel_m), ar_m)
        last_100.append(trans)
        if len(last_100) == 100:
            print(np.average(np.array(last_100), axis=0))
            last_100 = []

def main():
    global pub

    rospy.init_node('test')

    rospy.Subscriber('/tf', TFMessage, ar_callback)
    rospy.spin()

if __name__ == '__main__':
    main()