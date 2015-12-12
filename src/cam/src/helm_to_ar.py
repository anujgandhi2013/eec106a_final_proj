#!/usr/bin/env python
import numpy as np
import rospy
from kinematics import tf_to_rbt, rbt_to_pose, VICON_TO_CRAZY
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, Pose, Point, PoseArray, Quaternion

np.set_printoptions(precision=3)
pub = None

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

# AR_MAIN = 'ar_marker_1'
# AR_SIDE = 'ar_marker_0'
# mtag, stag = None, None
# last_100_m_to_s = []
# def ar_callback(msg):
#     global mtag, stag, last_100_m_to_s
#     for trans in msg.transforms:
#         if trans.child_frame_id == AR_MAIN:
#             mtag = trans.transform
#         elif trans.child_frame_id == AR_SIDE:
#             stag = trans.transform
#     if mtag is not None and stag is not None:
#         mtag_m = tf_to_rbt(mtag)
#         stag_m = tf_to_rbt(stag)
#         m_to_s = np.dot(np.linalg.inv(mtag_m), stag_m)
#         last_100_m_to_s.append(m_to_s)
#         if len(last_100_m_to_s) == 100:
#             m_to_s_avg = np.average(np.array(last_100_m_to_s), axis=0)
#             print(m_to_s_avg)
#             last_100_m_to_s = []

#             tl_m = np.dot(np.linalg.inv(m_to_s_avg), np.array([-.0381, .0381, 0, 1]))
#             bl_m = np.dot(np.linalg.inv(m_to_s_avg), np.array([-.0381, -.0381, 0, 1]))
#             tr_m = np.dot(np.linalg.inv(m_to_s_avg), np.array([.0381, .0381, 0, 1]))
#             br_m = np.dot(np.linalg.inv(m_to_s_avg), np.array([.0381, -.0381, 0, 1]))
#             print "tl_m:", tl_m
#             print "bl_m:", bl_m
#             print "tr_m:", tr_m
#             print "br_m:", br_m


def main():
    global pub

    rospy.init_node('test')
    pub = rospy.Publisher('tag_corners', PoseArray, queue_size=10)

    rospy.Subscriber('/tf', TFMessage, ar_callback)
    rospy.spin()

if __name__ == '__main__':
    main()