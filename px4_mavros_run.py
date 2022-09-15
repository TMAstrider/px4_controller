import rospy
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, Float64, String
import time
from pyquaternion import Quaternion
import math
import threading


class Px4Controller:
# 
    def __init__(self):

        self.imu = None
        self.gps = None
        self.local_pose = PoseStamped()
        self.local_pose.pose.position.z = 0.3
        self.current_state = None
        self.current_heading = None
        self.takeoff_height = 3.2
        self.local_enu_position = None

        self.cur_target_pose = None
        self.global_target = None

        self.received_new_task = False
        self.arm_state = False
        self.offboard_state = False
        self.received_imu = False
        self.frame = "BODY"

        self.state = None

        '''
        ros subscribers
        '''
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        # 时间戳 + 实时位置 + 实时朝向
        self.mavros_sub = rospy.Subscriber("/mavros/state", State, self.mavros_state_callback)
        # connection + Arming_status + Mode  状态  
        # self.gps_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_callback) 
        # 经度 + 纬度 + 海拔 + GPS 状态
        # self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback)
        # 返回四元数方向 旋转向量

        self.set_target_position_sub = rospy.Subscriber("gi/set_pose/position", PoseStamped, self.set_target_position_callback)
        # self.set_target_yaw_sub = rospy.Subscriber("gi/set_pose/orientation", Float32, self.set_target_yaw_callback)
        self.custom_activity_sub = rospy.Subscriber("gi/set_activity/type", String, self.custom_activity_callback)


        '''
        ros publishers
        '''
        # self.local_target_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        # 用于 Arm 
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        # 用于 设置 offboard Mode

        print("Px4 Controller Initialized!")


    def start(self):

        # 初始化节点
        rospy.init_node("offboard_node")
        # 等待 初始化
        for i in range(10):
            if self.current_heading is not None:
                break
            else:
                print("Waiting for initialization.")
                time.sleep(0.5)
        
        # self.cur_target_pose = self.construct_target(0, 0, self.takeoff_height, self.current_heading)
        self.cur_target_pose = self.construct_target(0, 0, self.takeoff_height)
        '''
        #print ("self.cur_target_pose:", self.cur_target_pose, type(self.cur_target_pose))

        '''
        rate = rospy.Rate(20)
        for i in range(100):   
            if(rospy.is_shutdown()):
                break

            self.local_pos_pub.publish(self.cur_target_pose)
            rate.sleep()
        # /*不停发布 arm请求*/       /*以及设置模式为 offboard*/
        for i in range(10):
            self.local_pos_pub.publish(self.cur_target_pose)
            self.arm_state = self.arm()
            self.offboard_state = self.offboard()
            time.sleep(0.1)

        # 起飞判断检测
        if self.takeoff_detection():
            print("Vehicle Took Off!")

        else:
            print("Vehicle Took Off Failed!")
            return
            ############################################
            #                                          #
            #             main ROS thread              #
            #                                          #
            ############################################
        '''
        main ROS thread
       
        在 Arm offboard_mode & not_rospy_is shutdown 下  
        '''

        while self.arm_state and self.offboard_state and (rospy.is_shutdown() is False):
            # 起飞到设定到初始位置
            self.local_pos_pub.publish(self.cur_target_pose)

            # 模式为 LAND mode & 实时高度 小于 0.15m  /* Disarm */
            if (self.state is "LAND") and (self.local_pose.pose.position.z < 0.15):

                if(self.disarm()):

                    self.state = "DISARMED"




    def construct_target(self, x, y, z):
        target_raw_pose = PoseStamped()
        target_raw_pose.header.stamp = rospy.Time.now()

        target_raw_pose.pose.position.x= x
        target_raw_pose.pose.position.y = y
        target_raw_pose.pose.position.z = z

        return target_raw_pose



    '''
    cur_p : poseStamped
    target_p: positionTarget
    '''



    def local_pose_callback(self, msg):
        self.local_pose = msg


    def mavros_state_callback(self, msg):
        self.mavros_state = msg.mode


    def set_target_position_callback(self, msg):

            self.cur_target_pose = self.construct_target(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)


    '''
     Receive A Custom Activity
     '''

    def custom_activity_callback(self, msg):

        print("Received Custom Activity:", msg.data)

        if msg.data == "LAND":
            print("LANDING!")
            self.state = "LAND"
            self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x,
                                                         self.local_pose.pose.position.y,
                                                         0.05)
                            

        if msg.data == "HOVER":
            print("HOVERING!")
            self.state = "HOVER"
            self.hover()

        else:
            print("Received Custom Activity:", msg.data, "not supported yet!")



    def arm(self):
        if self.armService(True):
            return True
        else:
            print("Vehicle arming failed!")
            return False

    def disarm(self):
        if self.armService(False):
            return True
        else:
            print("Vehicle disarming failed!")
            return False


    def offboard(self):
        if self.flightModeService(custom_mode='OFFBOARD'):
            return True
        else:
            print("Vechile Offboard failed")
            return False


    def hover(self):

        self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x,
                                                     self.local_pose.pose.position.y,
                                                     self.local_pose.pose.position.z,
                                                     self.current_heading)

    def takeoff_detection(self):
        if self.local_pose.pose.position.z > 0.1 and self.offboard_state and self.arm_state:
            return True
        else:
            return False


if __name__ == '__main__':


    
    con = Px4Controller()
    con.start()


