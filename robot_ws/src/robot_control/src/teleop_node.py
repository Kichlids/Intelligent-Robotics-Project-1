#! /usr/bin/env python

import rospy
import sys
import select
import termios
import tty

from robot_msgs.msg import keyboard


class KeyboardInput:

    def __init__(self):

        self.input_keys = ['w', 'a', 's', 'd']

        self.keyboard_pub = rospy.Publisher('/robot/keyboard_input', keyboard, queue_size = 10)
        
        self.construct_keyboard_msg()

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def construct_keyboard_msg(self):
        key = self.get_key()
        keyboard_msg = keyboard()

        if key in self.input_keys:
            keyboard_msg.is_teleop = True
            keyboard_msg.command = key
        else:
            keyboard_msg.is_teleop = False

        self.keyboard_pub.publish(keyboard_msg)


def init_teleop_node():
    rospy.init_node('teleop_node', anonymous = False)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        keyboard_input = KeyboardInput()

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)

    try:
        init_teleop_node()
    except rospy.ROSInterruptException:
        pass

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)