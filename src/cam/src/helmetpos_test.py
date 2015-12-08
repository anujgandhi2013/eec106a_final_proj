#!/usr/bin/env python
import rospy
import numpy as np
import kinematics as ks
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
from math import pi
from random import random

def main():
    rospy.init_node('helmet')
    pub = rospy.Publisher('helmet', TransformStamped, 10)

    tx, ty, tz = 1, 1, 1
    w0, w1, w2 = 0, 0, 1
    theta = pi/2
    r = rospy.Rate(1000)
    while True:
        # tx, ty, tz = tx+delt(), ty+delt(), tz+delt()
        # w0, w1, w2, = w0+delt(), w1+delt(), w2+delt()
        # theta = (theta+delt()) % (2*pi)
        
        quat = ks.rot_to_quaternion(ks.rotation_3d(np.array([w0, w1, w2]), theta))
        translation = Vector3(tx, ty, tz)
        rotation = Quaternion(*quat)
        transform = Transform(translation, rotation)
        ts = TransformStamped()
        ts.transform = transform

        pub.publish(ts)
        r.sleep()

def delt():
    return (random()-0.5)*.1

if __name__ == '__main__':
    main()

