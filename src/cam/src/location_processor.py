#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, Quaternion

pub = None

def pos_callback(pos):
    pub.publish(pos)

def main():
    global pub

    rospy.init_node('current_cf')
    pub = rospy.Publisher('copter_pos', Pose, queue_size=10)

    rospy.Subscriber('/copter_pos_raw', Pose, pos_callback)
    rospy.spin()

if __name__ == '__main__':
    main()