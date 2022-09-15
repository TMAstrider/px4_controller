

#! /usr/bin/env python

# from types import NoneType
from turtle import pos
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import time

# current_state = State()


class Px4Controller:


    def __init__(self):
        self.pose = None
        self.takeoff_x = 0
        self.takeoff_y = 0
        self.takeoff_height = 3
        self.current_state = None
        '''
        Ros subscribers
        '''
        self.state_sub = rospy.Subscriber("mavros/state", State, callback = self.state_cb)
        self.set_target_position_sub = rospy.Subscriber("gi/set_pose/position", PoseStamped, self.set_target_position_callback)

        '''
        Ros publishers
        '''
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        # self.local_target_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        

        '''
        Ros services
        '''
        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)


        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        
    def set_target_position_callback(self, msg):
        self.tar_pose = self.construct_target(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def state_cb(self, msg):
        # global current_state
        self.current_state = msg


    def start(self):
        rospy.init_node("offb_node_py")

        # Setpoint publishing MUST be faster than 2Hz
        rate = rospy.Rate(20)

        # Wait for Flight Controller connection
        while(not rospy.is_shutdown() and not self.current_state.connected):
            rate.sleep()

        self.tar_pose = self.construct_target(self.takeoff_x, self.takeoff_y, self.takeoff_height)

        # Send a few setpoints before starting
        for i in range(100):   
            if(rospy.is_shutdown()):
                break

            self.local_pos_pub.publish(self.tar_pose)
            rate.sleep()

        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        last_req = rospy.Time.now()

        while(not rospy.is_shutdown()):
            if(self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(self.set_mode_client.call(offb_set_mode).mode_sent == True):
                    rospy.loginfo("OFFBOARD enabled")
                
                last_req = rospy.Time.now()
            else:
                if(not self.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                    if(self.arm() == True):
                        rospy.loginfo("Vehicle armed")
                
                    last_req = rospy.Time.now()

            self.local_pos_pub.publish(self.tar_pose)

            rate.sleep()


    '''
    construct a target_pose for publisher
    '''
    def construct_target(self, x, y, z):
        target_pose = PoseStamped()
        target_pose.header.stamp = rospy.Time.now()
        # target_pose.coordinate_frame = 9
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z

        return target_pose

    # def move(self, x, y, z):
    #     self.position_target_pub.publish(self.set_pose(x, y, z))
    def arm(self):
        if self.armService(True):
            return True
        else:
            print("Vehicle arming failed!")
            return False
        


if __name__ == '__main__':
    con = Px4Controller()
    con.start()