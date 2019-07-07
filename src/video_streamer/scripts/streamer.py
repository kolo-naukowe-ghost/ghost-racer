#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from camera_streamer import CameraStreamer
from video_streamer import VideoStreamer

node_name = 'video_streamer'
loop_rate = 30
source_device = '/dev/video0'

target_fps = 30
target_width = 500
target_height = 500


def main():
	rospy.init_node(node_name, anonymous=True)

	cv_bridge = CvBridge()
	publisher = rospy.Publisher(VideoStreamer.topic_name, Image, queue_size=10)

	video_streamer = CameraStreamer(target_fps, target_width, target_height)

	loop_rate = video_streamer.get_fps()
	rate = rospy.Rate(loop_rate)

	while not rospy.is_shutdown():
		try:
			ret, frame = video_streamer.get_stream()
			if ret:
				message = cv_bridge.cv2_to_imgmsg(frame, encoding='passthrough')
				publisher.publish(message)
			else:
				rospy.logwarn('Failed to capture a frame.')

			rate.sleep()
		except rospy.ROSInterruptException:
			print("Exception occured")

	video_streamer.capture.release()


if __name__=='__main__':
	main()

