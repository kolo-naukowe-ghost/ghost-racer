#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from camera_streamer import CameraStreamer
from video_streamer import VideoStreamer
from csi_camera_streamer import CsiCameraStreamer

node_name = 'video_streamer'
loop_rate = 30
source_device = '/dev/video0'

target_fps = 60
target_width = 500
target_height = 500


def main():
    rospy.init_node(node_name, anonymous=True)

    cv_bridge = CvBridge()

    publisher = rospy.Publisher(VideoStreamer.topic_name, Image, queue_size=10)

    video_streamer = CsiCameraStreamer(target_fps, target_width, target_height)
    if not video_streamer.initialize_stream():
            rospy.logerr('Failed to initialize video stream')

    loop_rate = video_streamer.get_fps()
    rospy.loginfo("Starting stream, width: {}, height: {}, FPS: {}".format(video_streamer.w, video_streamer.h, loop_rate))
    rate = rospy.Rate(loop_rate)

    while not rospy.is_shutdown():
        try:
            ret, frame = video_streamer.get_stream()
            if ret:
                message = cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                message.header.stamp = rospy.get_rostime()
                publisher.publish(message)
            else:
                rospy.logwarn('Failed to capture a frame.')

            rate.sleep()
        except rospy.ROSInterruptException:
            print("Exception occured")

    video_streamer.capture.release()


if __name__=='__main__':
    main()

