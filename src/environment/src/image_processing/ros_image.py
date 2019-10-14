import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RosImage():
    def __init__(self, topic_name):

        self.topic_name = topic_name
        self.image_received_callback = "None"
        self.center_camera_topic = \
            rospy.Subscriber(self.topic_name, Image, self.image_callback)
        self.image = None

    def image_callback(self, raw_image):
        self.image = CvBridge().imgmsg_to_cv2(raw_image, "bgr8")
        if self.image_received_callback is not None and callable(self.image_received_callback):
            self.image_received_callback(raw_image)

    @staticmethod
    def is_image_valid(image):
        return image is not None