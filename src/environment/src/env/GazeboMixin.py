import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty


class GazeboMixin(object):
    def __init__(self):
        rospy.init_node('env')

        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.unpause_service = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause_service = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

    def unpause_gazebo(self):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause_service()
        except rospy.ServiceException:
            print("/gazebo/unpause_physics service call failed")

    def pause_gazebo(self):
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause_service()
        except rospy.ServiceException:
            print("/gazebo/pause_physics service call failed")

    def reset_gazebo(self):
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except rospy.ServiceException:
            print("/gazebo/reset_simulation service call failed")

    def publish_gazebo(self, message: Twist):
        self.cmd_vel.publish(message)
