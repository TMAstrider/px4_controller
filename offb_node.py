"""
 * File: offb_node.py
 * Stack and tested in Gazebo 9 SITL
"""

#! /usr/bin/env python

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

class px4_commander:

    def __init__(self):
        '''
        ros subscribers
        '''
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        # 时间戳 + 实时位置 + 实时朝向
        self.mavros_sub = rospy.Subscriber("/mavros/state", State, self.mavros_state_callback)
        # connection + Arming_status + Mode  状态  
        self.gps_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_callback) 
        # 经度 + 纬度 + 海拔 + GPS 状态
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback)
        # 返回四元数方向 旋转向量

        self.set_target_position_sub = rospy.Subscriber("gi/set_pose/position", PoseStamped, self.set_target_position_callback)
        self.set_target_yaw_sub = rospy.Subscriber("gi/set_pose/orientation", Float32, self.set_target_yaw_callback)
        self.custom_activity_sub = rospy.Subscriber("gi/set_activity/type", String, self.custom_activity_callback)


        '''
        ros publishers
        '''
        self.local_target_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        '''
        ros services
        '''


        print("Px4 Controller Initialized!")
        pass

    def start(self):
        # 初始化节点
        rospy.init_node("offboard_node")


        pose = PoseStamped()

        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 2

        for _ in range(10):
            self.local_target_pub.publish(self.cur_target_pose)
            self.arm_state = self.arm()
            self.offboard_state = self.offboard()
            time.sleep(0.2)

        # 起飞判断检测
        if self.takeoff_detection():
            print("Vehicle Took Off!")

        else:
            print("Vehicle Took Off Failed!")
            return

        pass
        '''
            main ROS thread
        
            在 Arm offboard_mode & not_rospy_is shutdown 下  
        '''
        while self.arm_state and self.offboard_state and (rospy.is_shutdown() is False):
            # 起飞到设定到初始位置
            self.local_target_pub.publish(self.cur_target_pose)

            # 模式为 LAND mode & 实时高度 小于 0.15m  /* Disarm */
            if (self.state is "LAND") and (self.local_pose.pose.position.z < 0.15):

                if(self.disarm()):

                    self.state = "DISARMED"


            time.sleep(0.1)


    def construct_target(self):
        pass
        
    
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




####################################################################################################################################
####################################################################################################################################
####################################################################################################################################
current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg


if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    # Send a few setpoints before starting
    for i in range(100):   
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            
            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
            
                last_req = rospy.Time.now()

        local_pos_pub.publish(pose)

        rate.sleep()