#! /usr/bin/env python

import rospy
import sys, select, termios, tty
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent

# Constants
LINEAR_SPEED_DEFAULT = 1
ANGULAR_SPEED_DEFAULT = 1

# Use WASD to move
input_keys = ['w', 'a', 's', 'd']

# Global variables
_can_teleop_control = True


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

    key = get_key()
    msg = Twist()

    if key in input_keys:
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

        return msg

    return None


def bumper_callback(data):
    global _can_teleop_control

    if data.state == BumperEvent.PRESSED:
        _can_teleop_control = False

def init_teleop_node():
    
    global _can_teleop_control

    rospy.init_node('teleop_node', anonymous = False)

    rate = rospy.Rate(10)
    teleop_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size = 10)
    bumper_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, bumper_callback)

    while not rospy.is_shutdown():
        if _can_teleop_control:
            msg = construct_teleop_msg()
            
            if msg != None:    
                teleop_pub.publish(msg)
                print(msg)

        rate.sleep()
        

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)

    try:
        init_teleop_node()
    except rospy.ROSInterruptException:
        pass

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)