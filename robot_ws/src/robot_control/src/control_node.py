#! /usr/bin/env python

import rospy
import sys, select, termios, tty

import random
import math

from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from sensor_msgs.msg import LaserScan

# Constants
LINEAR_SPEED_DEFAULT = 0.2
ANGULAR_SPEED_DEFAULT = 100

# Use WASD to move
input_keys = ['w', 'a', 's', 'd']

# Global variables

_velocity_pub = None
_collision_detected = False
_is_teleop_controlled = False



# Gets keyboard input from user
# Stole this from Turtlebot
def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# Given keyboard input construct appropriate robot control msg
def construct_teleop_msg():

    global _is_teleop_controlled

    key = get_key()

    if key in input_keys:
        msg = Twist()

        if key == 'w':
            msg.linear.x = LINEAR_SPEED_DEFAULT
        elif key == 'a':
            msg.angular.z = ANGULAR_SPEED_DEFAULT
        elif key == 's':
            msg.linear.x = -LINEAR_SPEED_DEFAULT
        elif key == 'd':
            msg.angular.z = -ANGULAR_SPEED_DEFAULT
        
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0

        _is_teleop_controlled = True
        return msg

    _is_teleop_controlled = False
    return None

def bumper_callback(data):
    global _collision_detected
    global _is_teleop_controlled

    if data.state == BumperEvent.PRESSED:
        _collision_detected = True
        _is_teleop_controlled = False

def move_autonomously(target_distance):

    global _velocity_pub

    # Forward movement
    '''
    move_msg = Twist()
    move_msg.linear.x = LINEAR_SPEED_DEFAULT

    t0 = rospy.Time.now().to_sec()
    current_distance = 0

    while (current_distance < target_distance):
        if _is_teleop_controlled:
            return
        
        _velocity_pub.publish(move_msg)
        t1 = rospy.Time.now().to_sec()
        current_distance = LINEAR_SPEED_DEFAULT * (t1 - t0)
    
    move_msg.linear.x = 0
    _velocity_pub.publish(move_msg)
    '''

    # Rotation movement

    turn_msg = Twist()
    turn_msg.angular.z = ANGULAR_SPEED_DEFAULT

    t0 = rospy.Time.now().to_sec()
    current_angle = 0
    target_angle = get_random_number(-15.0, 15.0)

    while (current_angle < target_angle):
        if (_is_teleop_controlled):
            return
        
        _velocity_pub.publish(turn_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = rad_to_deg(ANGULAR_SPEED_DEFAULT) * (t1 - t0)
    
    turn_msg.angular.z = 0
    _velocity_pub.publish(turn_msg)


def meters_to_feet(val):
    return val * 3.28

def get_random_number(min, max):
    rand = random.randint(min, max)
    print(rand)
    return rand

def rad_to_deg(rad):
    print
    return rad * 180 / math.pi


def init_control_node():

    global _velocity_pub
    global _collision_detected
    global _is_teleop_controlled
    

    rospy.init_node('control_node', anonymous = False)
    rate = rospy.Rate(10)

    _velocity_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size = 10)
    bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, bumper_callback)
    #laser_sub = rospy.Subscriber('/kobuki/laser/scan', LaserScan, laser_callback)

    while not rospy.is_shutdown():

        move_msg = construct_teleop_msg()

        if _collision_detected:
            no_movement_msg = Twist()
            _velocity_pub.publish(no_movement_msg)
        elif _is_teleop_controlled:
            _velocity_pub.publish(move_msg)
        
        else:
            move_autonomously(meters_to_feet(1))
        
        
        
        rate.sleep()


if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)

    try:
        init_control_node()
    except rospy.ROSInterruptException:
        pass

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)