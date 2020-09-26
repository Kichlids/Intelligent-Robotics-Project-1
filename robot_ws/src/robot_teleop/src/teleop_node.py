#! /usr/bin/env python

import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty


LINEAR_SPEED_DEFAULT = 1
ANGULAR_SPEED_DEFAULT = 1

# Use WASD to move
input_keys = ['w', 'a', 's', 'd']


# stole this from turtlebot
def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


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

def init_teleop_node():
    
    global _teleop_pub

    rospy.init_node('teleop_node', anonymous = False)
    rate = rospy.Rate(10)
    _teleop_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size = 10)

    while not rospy.is_shutdown():
        msg = construct_teleop_msg()
        
        if msg != None:    
            _teleop_pub.publish(msg)
            print(msg)

        rate.sleep()
        

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)

    try:
        init_teleop_node()
    except rospy.ROSInterruptException:
        pass

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)