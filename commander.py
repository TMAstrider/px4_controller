import rospy
from mavros_msgs.msg import GlobalPositionTarget, State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String
from pyquaternion import Quaternion
import time
import math


class Commander:
    def __init__(self):
        rospy.init_node("commander_node")
        rate = rospy.Rate(20)
        self.position_target_pub = rospy.Publisher('gi/set_pose/position', PoseStamped, queue_size=10)
        # self.yaw_target_pub = rospy.Publisher('gi/set_pose/orientation', Float32, queue_size=10)
        self.custom_activity_pub = rospy.Publisher('gi/set_activity/type', String, queue_size=10)

    # 俯仰滚转升降 按照body_ENU
    def move(self, x, y, z):
        self.position_target_pub.publish(self.set_pose(x, y, z))

    # 设置偏航角度
    # def turn(self, yaw_degree):
    #     self.yaw_target_pub.publish(yaw_degree)

    
    # land at current position
    # LAND MODE
    def land(self):
        self.custom_activity_pub.publish(String("LAND"))


    # hover at current position
    # HOVER MODE
    def hover(self):
        self.custom_activity_pub.publish(String("HOVER"))


    # return to home position with defined height
    # RETURN MODE
    def return_home(self, height):
        self.position_target_pub.publish(self.set_pose(0, 0, height, False))

    # 位置设置
    def set_pose(self, x=0, y=0, z=2):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()

        # ROS uses ENU internally, so we will stick to this convention
        # if BODY_FLU:
        #     pose.header.frame_id = 'base_link'

        # else:
        #     pose.header.frame_id = 'map'

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        return pose


if __name__ == "__main__":
    
    con = Commander()
    time.sleep(4)
    con.move(10, 0, 2)
    time.sleep(4)
    con.move(0, 10, 2)
    time.sleep(4)
    con.move(10, 10, 2)
    time.sleep(4)
    con.land()
    



