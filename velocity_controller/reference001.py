#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import math
from geometry_msgs.msg import PoseStamped,Quaternion,Twist
from mavros_msgs.srv import CommandBool,SetMode,CommandTOL
from mavros_msgs.msg import State
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler
from six.moves import xrange



state = State()
pos = PoseStamped()
vel = Twist()
time_out = 30
pos.header = Header()
yaw_degrees = 90
yaw = math.radians(yaw_degrees)
quaternion = quaternion_from_euler(0, 0, yaw)
pos.pose.orientation = Quaternion(*quaternion)


rospy.init_node('mavros_att', anonymous=True)


def state_of_drone(data):
    global current_state

    current_state = data
    #rospy.loginfo('state of Drone:  {0}'.format(data))




def set_arming(arm,time):

    rospy.loginfo('Setting arm: {0}'.format(arm))
    old_arm = state.armed
    loop_freq = 1
    rate = rospy.Rate(loop_freq)
    arm_set = False
    for i in range(time*loop_freq):
        if state.armed == arm:
            arm_set = True
            rospy.loginfo('set arm success | seconds: {0} of {1}'.format(i/loop_freq,time))
            break
        else:
            try:
                srv = set_arm(arm)
                if not srv.success:
                    rospy.logerr('Failed to send arm command')
            except rospy.ServiceException as e:
                rospy.logerr(e)
        try:
            rate.sleep()
        except rospy.ROSException as e:
            rospy.loginfo(e)



def setting_mode(mode,time):

    rospy.loginfo('Setting Mode : {0}'.format(mode))
    old_mode = state.mode
    loop_freq = 1
    rate = rospy.Rate(loop_freq)
    mode_set = False
    for i in range(time*loop_freq):
        if state.mode == mode:
            mode_set = True
            rospy.loginfo('set mode success | seconds: {0} of {1}'.format(i/loop_freq,time))
            break
        else:
            try:
                srv = set_mode(0,mode)
                if not srv.mode_sent:
                    rospy.logerr('failed to send mode command')
            except rospy.ServiceException as e:
                rospy.logerr(e)
        try:
            rate.sleep()
        except rospy.ROSException as e:
            rospy.loginfo(e)

def set_param(param_id, param_value, timeout):

    if param_value.integer != 0:
        value = param_value.integer
    else:
        value = param_value.real
        rospy.loginfo("setting PX4 parameter: {0} with value {1}".
                      format(param_id, value))
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        param_set = False
        for i in xrange(timeout * loop_freq):
            try:
                res = set_param_srv(param_id, param_value)
                if res.success:
                    rospy.loginfo("param {0} set to {1} | seconds: {2} of {3}".
                                  format(param_id, value, i / loop_freq, timeout))
                break
            except rospy.ServiceException as e:
                rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)


try:
    rospy.wait_for_service('mavros/set_mode',time_out)
    rospy.wait_for_service('mavros/cmd/arming',time_out)
    rospy.wait_for_service('mavros/cmd/takeoff',time_out)
    rospy.wait_for_service('mavros/param/set', time_out)
except rospy.ROSException:
    rospy.loginfo('fail to connect services')


rospy.Subscriber('mavros/state',State,state_of_drone)
pub_pos = rospy.Publisher('mavros/setpoint_position/local',PoseStamped,queue_size=10)
pub_vel = rospy.Publisher('mavros/setpoint_velocity/cmd_vel_unstamped',Twist,queue_size=10)

set_mode = rospy.ServiceProxy('mavros/set_mode',SetMode)
set_arm = rospy.ServiceProxy('mavros/cmd/arming',CommandBool)
takeoff = rospy.ServiceProxy('mavros/cmd/takeoff',CommandTOL)


rate = rospy.Rate(20)


pos.header.frame_id = 'map'
pos.pose.position.x = 0
pos.pose.position.y = 0
pos.pose.position.z = 5

vel.linear.x = -5.0
vel.linear.y = 0.0
vel.linear.z = 0.0
vel.angular.x = 10.0
vel.angular.y = 0.0
vel.angular.z = 0.0

setting_mode('GUIDED',5)
set_arming(True,5)
takeoff(0,0,0,0,5)

for i in range(100):
    i = i+1
    rate.sleep()

while not rospy.is_shutdown():
    pos.header.stamp = rospy.Time.now()
    #pub_pos.publish(pos)
    pub_vel.publish(vel)
    try:
        rate.sleep()
    except rospy.ROSInterruptException:
        pass
