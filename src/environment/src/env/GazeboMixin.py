import rospy
from cv_bridge import CvBridge
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Twist
from rospy import ROSException
from sensor_msgs.msg import Image
from std_srvs.srv import Empty


class GazeboMixin(object):
    def __init__(self):
        rospy.init_node('env')

        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.unpause_service = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause_service = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

    def _unpause_gazebo(self):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause_service()
        except rospy.ServiceException:
            print("/gazebo/unpause_physics service call failed")

    def _pause_gazebo(self):
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause_service()
        except rospy.ServiceException:
            print("/gazebo/pause_physics service call failed")

    def _reset_gazebo(self):
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except rospy.ServiceException:
            print("/gazebo/reset_simulation service call failed")

    def _publish_gazebo(self, message):
        """

        :param message: Twist
        :return: None
        """
        self.cmd_vel.publish(message)

    def _get_model_states(self):
        """
        for more information type: rosservice call /gazebo/get_model_state
        empty string in model_state is very important because
        it means that you want to state of whole model, not only specific link
        :return:
        """
        model_state = self.model_state('conde', '')
        return model_state

    @staticmethod
    def _get_image_data_from_topic(topic):
        image = None
        while image is None:
            try:
                image = rospy.wait_for_message(topic, Image, timeout=5)
                image = CvBridge().imgmsg_to_cv2(image, "bgr8")
            except ROSException:
                return None
        return image
