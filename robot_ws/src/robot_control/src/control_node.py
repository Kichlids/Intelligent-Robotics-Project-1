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
LINEAR_SPEED_DEFAULT = 0.5
ANGULAR_SPEED_DEFAULT = 0.4
AUTONOMOUS_FORWARD_DISTANCE = 1
# 0.449999988079 meters is the minimum dist detected, which is about 1.5 feet
LASER_AVOIDANCE_DISTANCE = 1.5
#LASER_CENTER_INDEX_THRESHOLD = 40
LASER_SYMMETRIC_VALUE_THRESHOLD = 0.3

'''
# Global variables
_velocity_pub = None

_vel_msg = Twist()

_collision_detected = False
_is_teleop_controlled = False

_symmetric_obstacle_detected = False
_asymmetric_obstacle_detected = False
_laser_data = None
_laser_min_index = -1

# Convert meters into feet
def meters_to_feet(val):
    return val * 3.28

def feet_to_meters(val):
    return val / 3.28

# Convert radians to degrees
def rad_to_deg(rad):
    return rad * 180 / math.pi

# Return a random number between min and max
def get_random_number(min, max):
    return random.uniform(min, max)

# Return minimum distance detected by laser that is bounded
# by range_min and range_max
def find_min_laser_data(array, range_min, range_max):
    min_val = array[0]
    min_index = 0
    for i in range(len(array)):
        #if min_val > range_min and min_val < range_max:
        if min_val > array[i]:
            min_val = array[i]
            min_index = i
    
    return min_val, min_index

def escape():
    global _velocity_pub
    global _is_teleop_controlled

    # Rotate 180 degrees
    t0 = rospy.Time.now().to_sec()
    current_angle = 0
    target_angle = 180

    turn_msg = Twist()
    turn_msg.angular.z = ANGULAR_SPEED_DEFAULT

    while (current_angle < abs(target_angle)):
        if _is_teleop_controlled:
            return
        
        _velocity_pub.publish(turn_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = rad_to_deg(ANGULAR_SPEED_DEFAULT) * (t1 - t0)

def avoid():
    global _velocity_pub
    global _is_teleop_controlled
    global _symmetric_obstacle_detected
    global _asymmetric_obstacle_detected
    global _laser_data
    global _laser_min_index

    turn_msg = Twist()
    # Obstacle found left side. Turn right
    if _laser_min_index < (len(_laser_data.ranges) - 1) / 2 :
        turn_msg.angular.z = ANGULAR_SPEED_DEFAULT
    # Obstacle found right side. Turn left
    else:
        turn_msg.angular.z = -ANGULAR_SPEED_DEFAULT

    while (_asymmetric_obstacle_detected):
        if _is_teleop_controlled or _symmetric_obstacle_detected:
            return
        
        _velocity_pub.publish(turn_msg)

def move_autonomously():

    global _velocity_pub

    # Forward movement
    move_msg = Twist()
    move_msg.linear.x = feet_to_meters(LINEAR_SPEED_DEFAULT)

    t0 = rospy.Time.now().to_sec()
    current_distance = 0

    print('Moving forward 1ft')
    while (current_distance < AUTONOMOUS_FORWARD_DISTANCE):
        if _is_teleop_controlled or _asymmetric_obstacle_detected or _symmetric_obstacle_detected:
            return
        
        _velocity_pub.publish(move_msg)
        t1 = rospy.Time.now().to_sec()
        current_distance = LINEAR_SPEED_DEFAULT * (t1 - t0)

    # Rotation movement
    t0 = rospy.Time.now().to_sec()
    current_angle = 0
    target_angle = get_random_number(-15.0, 15.0)
    
    print('Rotating ' + str(round(target_angle, 2)))

    turn_msg = Twist()
    if target_angle >= 0:
        turn_msg.angular.z = ANGULAR_SPEED_DEFAULT
    else:
        turn_msg.angular.z = -ANGULAR_SPEED_DEFAULT


    while (current_angle < abs(target_angle)):
        if _is_teleop_controlled or _asymmetric_obstacle_detected or _symmetric_obstacle_detected:
            return
        
        _velocity_pub.publish(turn_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = rad_to_deg(ANGULAR_SPEED_DEFAULT) * (t1 - t0)

# Callback function in response to keyboard inputs
# Detects:
# - if user is controlling the robot
# - the command
def keyboard_callback(data):
    global _is_teleop_controlled
    global _vel_msg

    if data.is_teleop:
        if data.command == 'w':
            _vel_msg.linear.x = feet_to_meters(LINEAR_SPEED_DEFAULT)
        elif data.command == 'a':
            _vel_msg.angular.z = feet_to_meters(ANGULAR_SPEED_DEFAULT)
        elif data.command == 's':
            _vel_msg.linear.x = feet_to_meters(-LINEAR_SPEED_DEFAULT)
        else:
            _vel_msg.angular.z = feet_to_meters(-ANGULAR_SPEED_DEFAULT)
    else:
        _vel_msg = Twist()

    _is_teleop_controlled = data.is_teleop

# Callback function in response to collision
# Detects:
# - Any bumper has been triggered
def bumper_callback(data):
    global _collision_detected
    global _is_teleop_controlled

    if data.state == BumperEvent.PRESSED:
        _collision_detected = True
        _is_teleop_controlled = False

# Callback function in response to laser sensor
# Detects:
# - 720 samples of distance from robot to obstacles ranging from 0-180 degrees
#
# The length of list: ranges is 720. Index 0 represent 0 deg, 360 = 90, 720 = 180
def laser_callback(data):
    #print(data.ranges)
    
    global _symmetric_obstacle_detected
    global _asymmetric_obstacle_detected
    global _laser_data
    global _laser_min_index
    '''
    print(len(data.ranges))
    print(data.angle_increment)
    print(data.range_min)
    print(data.range_max)
    print(data.angle_min)
    print(data.angle_max)
    '''
    #print(meters_to_feet(data.ranges[len(data.ranges)/2]))

    
    ranges = []
    for i in range(len(data.ranges)):
        ranges.append(meters_to_feet(data.ranges[i]))
    
    range_min = meters_to_feet(data.range_min)
    range_max = meters_to_feet(data.range_max)

    min_val, min_index = find_min_laser_data(ranges, range_min, range_max)
   
    if math.isnan(min_val) or min_val < LASER_AVOIDANCE_DISTANCE:
        

        i = len(data.ranges) - 1 - min_index
        val_i = meters_to_feet(data.ranges[i])
        #print(str(min_val) + ', ' + str(val_i))
        #print(str(min_val) + ", " + str(0.09375 * min_index - 30) + ", " + str(val_i))

        if math.isnan(val_i) or abs(min_val - val_i) < LASER_SYMMETRIC_VALUE_THRESHOLD:
            # SYMMETTRIC
            _symmetric_obstacle_detected = True
            _asymmetric_obstacle_detected = False
        else:
            # ASSYMETRIC
            _symmetric_obstacle_detected = False
            _asymmetric_obstacle_detected = True
        
        _laser_data = data
        _laser_min_index = min_index

    else:
        _symmetric_obstacle_detected = False
        _asymmetric_obstacle_detected = False
    
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
    laser_sub = rospy.Subscriber('/scan', LaserScan, laser_callback)

    while not rospy.is_shutdown():

        if _collision_detected:
            print('COLLISION DETECTED: STOPPING...')
            no_movement_msg = Twist()
            _velocity_pub.publish(no_movement_msg)
            #return
        elif _is_teleop_controlled:
            #print('Teleop control detected')
            _velocity_pub.publish(_vel_msg)
        
        elif _symmetric_obstacle_detected:
            print('Encountered symmetric obstacle')
            escape()
        elif _asymmetric_obstacle_detected:
            print('Encountered asymmetric obstacle')
            avoid()
        else:
            move_autonomously()
        
        
        rate.sleep()

if __name__ == '__main__':
    try:
        init_control_node()
    except rospy.ROSInterruptException:
        pass


'''
# vel_msg
# is_teleop_controlled
class Keyboard():

    def __init__(self):
        self.is_teleop_controlled = False
        self.vel_msg = Twist()

        self.keyboard_sub = rospy.Subscriber('/robot/keyboard_input', keyboard, keyboard_callback)

    def keyboard_callback(self, data):
        self.vel_msg = Twist()

        if data.is_teleop:
            if data.command == 'w':
                vel_msg.linear.x = feet_to_meters(LINEAR_SPEED_DEFAULT)
            elif data.command == 'a':
                vel_msg.angular.z = feet_to_meters(ANGULAR_SPEED_DEFAULT)
            elif data.command == 's':
                vel_msg.linear.x = feet_to_meters(-LINEAR_SPEED_DEFAULT)
            else:
                vel_msg.angular.z = feet_to_meters(-ANGULAR_SPEED_DEFAULT)
        
        self.is_teleop_controlled = data.is_teleop

# collision_detected
class Bumper():

    def __init__(self):
        self.collision_detected = False
        
        self.bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumper_callback)

    def bumper_callback(self, data):
        if data.state == BumperEvent.PRESSED:
            self.collision_detected = True

# symmetric_obstacle_detected
# asymmetric_obstacle_detected
# laser_data
# laser_min_index
class Laser():

    def __init__(self):
        self.support = Support()

        self.symmetric_obstacle_detected = False
        self.asymmetric_obstacle_detected = False
        self.laser_data
        self.laser_min_index

        self.laser_sub = rospy.Subscriber('/scan', LaserScan, laser_callback)

    def find_min_laser_data(self, array, range_min, range_max):
        min_val = array[0]
        min_index = 0
        for i in range(len(array)):
            if min_val > array[i]:
                min_val = array[i]
                min_index = i
        
        return min_val, min_index
    
    def laser_callback(self, data):

        ranges = []
        for i in range(len(data.ranges)):
            ranges.append(meters_to_feet(data.ranges[i]))
        
        range_min = self.support.meters_to_feet(data.range_min)
        range_max = self.support.meters_to_feet(data.range_max)

        min_val, min_index = self.find_min_laser_data(ranges, range_min, range_max)
    
        if math.isnan(min_val) or min_val < LASER_AVOIDANCE_DISTANCE:
            

            i = len(data.ranges) - 1 - min_index
            val_i = self.support.meters_to_feet(data.ranges[i])

            if math.isnan(val_i) or abs(min_val - val_i) < LASER_SYMMETRIC_VALUE_THRESHOLD:
                # SYMMETTRIC
                self.symmetric_obstacle_detected = True
                self.asymmetric_obstacle_detected = False
            else:
                # ASSYMETRIC
                self.symmetric_obstacle_detected = False
                self.asymmetric_obstacle_detected = True
            
            self.laser_data = data
            self.laser_min_index = min_index

        else:
            self.symmetric_obstacle_detected = False
            self.asymmetric_obstacle_detected = False

class Support():

    def meters_to_feet(self, val):
        return val * 3.28

    def feet_to_meters(self, val):
        return val / 3.28

    def rad_to_deg(self, rad):
        return rad * 180 / math.pi

    def get_random_number(self, min, max):
        return random.uniform(min, max)

class Movement():

    def __init__(self, keyboard, laser):
        self.keyboard = keyboard
        self.laser = laser
        self.support = Support()
        self.velocity_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size = 10)

    def velocity_publish(self, vel_msg):
        self.velocity_pub.publish(vel_msg)

    def autonomous(self):
        
        # Moving forward
        move_msg = Twist()
        move_msg.linear.x = self.support.feet_to_meters(LINEAR_SPEED_DEFAULT)

        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        print('Moving forward 1ft')
        while (current_distance < AUTONOMOUS_FORWARD_DISTANCE):
            if keyboard.is_teleop_controlled or laser.asymmetric_obstacle_detected or laser.symmetric_obstacle_detected:
                return
        
            self.velocity_publish(move_msg)
            t1 = rospy.Time.now().to_sec()
            current_distance = LINEAR_SPEED_DEFAULT * (t1 - t0)
        
        # Rotating
        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        target_angle = self.support.get_random_number(-15.0, 15.0)
        
        print('Rotating ' + str(round(target_angle, 2)))

        turn_msg = Twist()
        if target_angle >= 0:
            turn_msg.angular.z = ANGULAR_SPEED_DEFAULT
        else:
            turn_msg.angular.z = -ANGULAR_SPEED_DEFAULT


        while (current_angle < abs(target_angle)):
            if self.keyboard.is_teleop_controlled or self.laser.asymmetric_obstacle_detected
                                                    or self.laser.symmetric_obstacle_detected:
                return
            
            self.velocity_publish(turn_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = self.support.rad_to_deg(ANGULAR_SPEED_DEFAULT) * (t1 - t0)        

    def escape(self):
        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        target_angle = 180

        turn_msg = Twist()
        turn_msg.angular.z = ANGULAR_SPEED_DEFAULT

        while (current_angle < abs(target_angle)):
            if keyboard.is_teleop_controlled:
               return

            self.velocity_publish(turn_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = self.support.rad_to_deg(ANGULAR_SPEED_DEFAULT) * (t1 - t0)

    def avoid(self):

        turn_msg = Twist()

        if laser.laser_min_index < (len(laser.laser_data.ranges) - 1) / 2:
            turn_msg.angular.z = ANGULAR_SPEED_DEFAULT
        else:
            turn_msg.angular.z = -ANGULAR_SPEED_DEFAULT
        
        while laser.asymmetric_obstacle_detected:
            if keyboard.is_teleop_controlled or laser.symmetric_obstacle_detected:
                return
            
            self.velocity_publish(turn_msg)


if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous = False)
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            print('hello')

        
        keyboard = Keyboard()
        bumper = Bumper()
        laser = Laser()
        
        movement = Movement(keyboard, laser)

        while not rospy.is_shutdown():
            if bumper.collision_detected:
                print('COLLISION DETECTED: STOPPING...')
                vel_msg = Twist()
                movement.velocity_publish(vel_msg)
            
            else if keyboard.is_teleop_controlled:
                print('Teleop control detected')
                movement.velocity_publish(keyboard.vel_msg)
            else if laser.symmetric_obstacle_detected:
                print('Encountered symmetric obstacle')
                movement.escape()
            elif laser.asymmetric_obstacle_detected:
                print('Encountered asymmetric obstacle')
                movement.avoid()
            
            else:
                #movement.autonomous()
                print('its fine')
        

    except rospy.ROSInterruptException:
        pass
