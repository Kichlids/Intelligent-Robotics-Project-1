#! /usr/bin/env python

import rospy
import sys, select, termios, tty
from geometry_msgs.msg import Twist
from robot_msgs.msg import keyboard

input_keys = ['w', 'a', 's', 'd']

def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def init_teleop_node():
    
    rospy.init_node('teleop_node', anonymous = False)
    rate = rospy.Rate(10)

    keyboard_pub = rospy.Publisher('/robot/keyboard_input', keyboard, queue_size = 10)

    while not rospy.is_shutdown():
        
        key = get_key()
        keyboard_msg = keyboard()

        if key in input_keys:
            keyboard_msg.is_teleop = True
            keyboard_msg.command = key
        else:
            keyboard_msg.is_teleop = False

        keyboard_pub.publish(keyboard_msg)
        
        rate.sleep()
        

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)

    try:
        init_teleop_node()
    except rospy.ROSInterruptException:
        pass

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)