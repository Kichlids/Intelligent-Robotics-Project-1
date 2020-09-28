#! /usr/bin/env python

import rospy
import random
import math
import numpy

from robot_msgs.msg import keyboard
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from sensor_msgs.msg import LaserScan

# Constants
LINEAR_SPEED_DEFAULT = 0.2
ANGULAR_SPEED_DEFAULT = 0.3
AUTONOMOUS_FORWARD_DISTANCE = 1
LASER_AVOIDANCE_DISTANCE = 1
LASER_CENTER_INDEX_THRESHOLD = 40
LASER_SYMMETRIC_VALUE_THRESHOLD = 0.2


# Global variables
_velocity_pub = None

_vel_msg = Twist()

_collision_detected = False
_is_teleop_controlled = False

_symmetric_obstacle_detected = False
_asymmetric_obstacle_detected = False
_laser_data = None

def meters_to_feet(val):
    return val / 3.28

def rad_to_deg(rad):
    return rad * 180 / math.pi

def get_random_number(min, max):
    return random.uniform(min, max)

def find_min_laser_data(data, range_min, range_max):
    min_val = math.inf
    min_index = -1
    for i in len(array):
        if min_val > range_min and min_val < range_max:
            if min_val > array[i]:
                min_val = array[i]
                min_index = i
    
    return min_val, min_index


def move_autonomously():

    global _velocity_pub

    # Forward movement
    move_msg = Twist()
    move_msg.linear.x = LINEAR_SPEED_DEFAULT

    t0 = rospy.Time.now().to_sec()
    current_distance = 0

    print('Moving forward 1ft')
    while (current_distance < AUTONOMOUS_FORWARD_DISTANCE):
        if _is_teleop_controlled:
            return
        
        _velocity_pub.publish(move_msg)
        t1 = rospy.Time.now().to_sec()
        current_distance = LINEAR_SPEED_DEFAULT * (t1 - t0)

    # Rotation movement
    t0 = rospy.Time.now().to_sec()
    current_angle = 0
    target_angle = get_random_number(-15.0, 15.0)
    
    print('Rotating ' + str(target_angle))

    turn_msg = Twist()
    if target_angle >= 0:
        turn_msg.angular.z = ANGULAR_SPEED_DEFAULT
    else:
        turn_msg.angular.z = -ANGULAR_SPEED_DEFAULT


    while (current_angle < abs(target_angle)):
        if (_is_teleop_controlled):
            return
        
        _velocity_pub.publish(turn_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = rad_to_deg(ANGULAR_SPEED_DEFAULT) * (t1 - t0)

def keyboard_callback(data):
    global _is_teleop_controlled
    global _vel_msg

    if data.is_teleop:
        if data.command == 'w':
            _vel_msg.linear.x = LINEAR_SPEED_DEFAULT
        elif data.command == 'a':
            _vel_msg.angular.z = ANGULAR_SPEED_DEFAULT
        elif data.command == 's':
            _vel_msg.linear.x = -LINEAR_SPEED_DEFAULT
        else:
            _vel_msg.angular.z = -ANGULAR_SPEED_DEFAULT
    else:
        _vel_msg = Twist()

    _is_teleop_controlled = data.is_teleop

# Callback function for collision detection
def bumper_callback(data):
    global _collision_detected
    global _is_teleop_controlled

    if data.state == BumperEvent.PRESSED:
        print('COLLISION DETECTED\n')
        _collision_detected = True
        _is_teleop_controlled = False

# The length of list: ranges is 720. Index 0 represent 0 deg, 360 = 90, 720 = 180
def laser_callback(data):
    global _symmetric_obstacle_detected
    global _asymmetric_obstacle_detected
    global _laser_data

    ranges = meters_to_feet(data.ranges)
    range_min = meters_to_feet(data.range_min)
    range_max = meters_to_feet(data.range_max)

    min_val, min_index = find_min_laser_data(ranges, range_min, range_max)
    
    if min_val < LASER_AVOIDANCE_DISTANCE:

        i = len(data.ranges) - 1 - min_index
        val_i = meters_to_feet(data.ranges[i])

        if abs(min_val - val_i) < LASER_SYMMETRIC_VALUE_THRESHOLD:
            # SYMMETTRIC
            _symmetric_obstacle_detected = True
            _asymmetric_obstacle_detected = False
        else:
            # ASSYMETRIC
            _symmetric_obstacle_detected = False
            _asymmetric_obstacle_detected = True
        
        _laser_data = data

    else:
        _symmetric_obstacle_detected = False
        _asymmetric_obstacle_detected = False

    '''
    if (meters_to_feet(min_val) <= 1):
        # Turn to the right
        if min_index < len(data.ranges) / 2:
            #
        # Turn to the left
        else:
            #
    '''

def init_control_node():

    global _vel_msg
    global _velocity_pub
    global _collision_detected
    global _is_teleop_controlled
    global _symmetric_obstacle_detected
    global _asymmetric_obstacle_detected
    

    rospy.init_node('control_node', anonymous = False)
    rate = rospy.Rate(10)

    keyboard_sub = rospy.Subscriber('/robot/keyboard_input', keyboard, keyboard_callback)
    _velocity_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size = 10)
    bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, bumper_callback)
    laser_sub = rospy.Subscriber('/kobuki/laser/scan', LaserScan, laser_callback)

    while not rospy.is_shutdown():

        if _collision_detected:
            print('COLLISION DETECTED: STOPPING...')
            no_movement_msg = Twist()
            _velocity_pub.publish(no_movement_msg)
            #return
        elif _is_teleop_controlled:
            print('Teleop control detected')
            _velocity_pub.publish(_vel_msg)
        elif _symmetric_obstacle_detected:
            print('Encountered symmetric obstacle')
        elif _asymmetric_obstacle_detected:
            print('Encountered asymmetric obstacle')
        else:
            move_autonomously()
        
        rate.sleep()


if __name__ == '__main__':
    try:
        init_control_node()
    except rospy.ROSInterruptException:
        pass
