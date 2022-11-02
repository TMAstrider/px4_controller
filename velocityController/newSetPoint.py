

#! /usr/bin/env python

# from types import NoneType
from itertools import zip_longest
from re import X
from turtle import pos, position
import rospy
from geometry_msgs.msg import PoseStamped 
from geometry_msgs.msg import Twist
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
        self.local_pos = PoseStamped()
        self.err_x, self.err_x0, self.err_x_err = 0, 0, 0
        self.err_y, self.err_y0, self.err_y_err = 0, 0, 0
        self.err_z, self.err_z0, self.err_z_err = 0, 0, 0

        self.vel_x, self.vel_y, self.vel_z = 0, 0, 0
        self.adj_kp = 1.0
        self.adj_kd = 1.88
        self.vec = None
        self.tar_pose_oriented = None



        '''
        Ros subscribers
        '''
        self.state_sub = rospy.Subscriber("mavros/state", State, callback = self.state_cb)
        self.set_target_position_sub = rospy.Subscriber("gi/set_pose/position", PoseStamped, self.set_target_position_callback)
        self.local_pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.local_pos_cb)
        self.position_tele_sub = rospy.Subscriber('gi/set_pose/tele', PoseStamped, self.tele_cb)

        '''
        Ros publishers
        '''
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        # self.local_target_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
        self.position_target_pub = rospy.Publisher('gi/set_pose/position', PoseStamped, queue_size=10)
        self.position_tele_pub = rospy.Publisher('gi/set_pose/tele', PoseStamped, queue_size=10)

        

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
        self.current_state = msg

    def local_pos_cb(self, msg):
        self.local_pos = msg
        pass
    
    def tele_cb(self, msg):
        self.local_pos_control(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        self.vel_pub.publish(self.vec)
        pass

    '''
    main pos control Func via velocity
    '''
    def local_pos_control(self, posix_x, posix_y, posix_z):
        self.err_x = posix_x - self.local_pos.pose.position.x
        self.err_y = posix_y - self.local_pos.pose.position.y
        self.err_z = posix_z - self.local_pos.pose.position.z

        self.err_x_err = self.err_x - self.err_x0
        self.err_y_err = self.err_y - self.err_y0
        self.err_z_err = self.err_z - self.err_z0

        # save the deviation
        self.err_x0 = self.err_x
        self.err_y0 = self.err_y
        self.err_z0 = self.err_z

        # print('DEBUG: err_x = ', self.err_x)
        # print('DEBUG: err_y = ', self.err_y)

        self.vel_x = self.adj_kp * self.err_x + self.adj_kd * self.err_x_err
        self.vel_y = self.adj_kp * self.err_y + self.adj_kd * self.err_y_err
        self.vel_z = self.adj_kp * self.err_z + self.adj_kd * self.err_z_err

        self.vec.linear.x = self.vel_x
        self.vec.linear.y = self.vel_y
        self.vec.linear.z = self.vel_z

   
        # rospy.loginfo("Pose-x: {0}".format(self.local_pos.pose.position.x))
        # rospy.loginfo("Pose-y: {0}".format(self.local_pos.pose.position.y))
        # rospy.loginfo("Pose-z: {0}".format(self.local_pos.pose.position.z))




    def start(self):
        rospy.init_node("offb_node_py")

        # Setpoint publishing MUST be faster than 2Hz
        rate = rospy.Rate(20)

        # Wait for Flight Controller connection
        while(not rospy.is_shutdown() and not self.current_state.connected):
            rate.sleep()

        self.tar_pose = self.construct_target(self.takeoff_x, self.takeoff_y, self.takeoff_height)
        
        # Initialize vec
        self.vec = Twist()
        self.vec.linear.x = 0
        self.vec.linear.y = 0
        self.vec.linear.z = 0

        self.tar_pose_oriented = PoseStamped()


        # Send a few setpoints before starting
        for i in range(100):   
            if(rospy.is_shutdown()):
                break

            self.vel_pub.publish(self.vec)
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

            # self.local_pos_pub.publish(self.tar_pose)
            self.position_tele_pub.publish(self.tar_pose)
            # self.vel_pub.publish(self.vec)

            rate.sleep()


    '''
    construct a target_pose for publisher
    '''
    def construct_target(self, x, y, z):
        target_pose = PoseStamped()
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
