#!/usr/bin/env python
from __future__ import print_function
import rospy
from tf2_msgs.msg import TFMessage

HELMET_TAG = 'ar_marker_0'


def callback(msg, ar_tag):
	for trans in msg.transforms:
		print(trans)

def main():
	rospy.init_node('current_cf')

	rospy.Subscriber('/tf', TFMessage, callback, HELMET_TAG)
	rospy.spin()

if __name__ == '__main__':
	main()