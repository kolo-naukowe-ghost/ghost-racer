#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from video_streamer.srv import *
import cv2

def get_video_stream(dummy):
	cap = cv2.VideoCapture(0)
	ret, frame = cap.read()
	return frame

def init_streamer_service():
	rospy.init_node('video_streamer')
	service = rospy.Service('video_streamer_service', VideoFrame, get_video_stream)
	print("initialized.")
	rospy.spin()

def streamer():
	pub = rospy.Publisher('video', String, queue_size=10)
	rospy.init_node('video_streamer', anonymous=True)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		hello_str = 'hello world {}'.format(rospy.get_time())
		rospy.loginfo(hello_str)
		pub.publish(hello_str)
		rate.sleep()

if __name__=='__main__':
	try:
		init_streamer_service()
		# streamer()
	except rospy.ROSInterruptException:
		print("Exception occured")

