#! /usr/bin/env python

import rospy
import random
import math
import numpy

from robot_msgs.msg import keyboard
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from sensor_msgs.msg import LaserScan


###### Constant variables ############

# Speed ft/s
LINEAR_SPEED_DEFAULT = 0.5
# Rotation speed ft/s
ANGULAR_SPEED_DEFAULT = 0.4
# Distance in ft to move forward in auto-move
AUTONOMOUS_FORWARD_DISTANCE = 1
# Obstacle avoidance threshold in ft, including the position of the laser scan sensor
LASER_AVOIDANCE_DISTANCE = 1.5
# Symmetric determining threshold in m
LASER_SYMMETRIC_VALUE_THRESHOLD = 0.3


# Subscribes to keyboard input and sets:
# - velocity message (vel_msg)
# - whether robot is controlled by user (is_teleop_controlled)
class Keyboard():

    def __init__(self):
        self.support = Support()

        self.is_teleop_controlled = False
        self.vel_msg = Twist()

        self.keyboard_sub = rospy.Subscriber('/robot/keyboard_input', keyboard, self.keyboard_callback)

    # Keyboard input callback function
    def keyboard_callback(self, data):
        self.vel_msg = Twist()

        if data.is_teleop:
            if data.command == 'w':
                self.vel_msg.linear.x = self.support.feet_to_meters(LINEAR_SPEED_DEFAULT)
            elif data.command == 'a':
                self.vel_msg.angular.z = self.support.feet_to_meters(ANGULAR_SPEED_DEFAULT)
            elif data.command == 's':
                self.vel_msg.linear.x = self.support.feet_to_meters(-LINEAR_SPEED_DEFAULT)
            else:
                self.vel_msg.angular.z = self.support.feet_to_meters(-ANGULAR_SPEED_DEFAULT)
        
        self.is_teleop_controlled = data.is_teleop


# Subscribes to Bumper sensor and sets:
# - whether collision has been detected (collisio_detected)
class Bumper():

    def __init__(self):
        self.collision_detected = False
        
        self.bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumper_callback)

    # Bumper callback function
    def bumper_callback(self, data):
        if data.state == BumperEvent.PRESSED:
            self.collision_detected = True


# Subscribes to the laser scanner sensor and sets:
# - whether symmetric obstacles are detected (symmetric_obstacle_detected)
# - whether asymmetric obstacles are detected (asymmetric_obstacles_detected)
# Also saves the LaserScan data received as well as the minimum index of minimum distance
class Laser():

    def __init__(self):
        self.support = Support()

        self.symmetric_obstacle_detected = False
        self.asymmetric_obstacle_detected = False
        self.laser_data = LaserScan()
        self.laser_min_index = 0

        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)

    # Find the minimum distance and its index of the ranges
    def find_min_laser_data(self, array):
        min_val = array[0]
        min_index = 0
        for i in range(len(array)):
            if min_val > array[i]:
                min_val = array[i]
                min_index = i
        
        return min_val, min_index
    
    # Laser Scanner callback function
    def laser_callback(self, data):

        #print(self.support.meters_to_feet(data.ranges[320]))

        # Convert ranges from meters to feet
        ranges = []
        for i in range(len(data.ranges)):
            ranges.append(self.support.meters_to_feet(data.ranges[i]))
        
        #range_min = self.support.meters_to_feet(data.range_min)
        #range_max = self.support.meters_to_feet(data.range_max)

        # Find minimum distance and index
        min_val, min_index = self.find_min_laser_data(ranges)

        # Object detected if minimum distance is less than threshold 
        # or too close to read (nan)
        # Assume (nan) values are below threshold, as:
        # - (nan) values can either be less than range_min
        # -             "              greater than range_max
        # range_max is about 30ft. It is impossible to get reading of 30ft+ given our environment
        # So the only other possible outcome is if distance read (nan) is below range_min, or threshold
        if math.isnan(min_val) or min_val < LASER_AVOIDANCE_DISTANCE:
            
            # Compare the minimum distance to the value mirrored at center
            i = len(data.ranges) - 1 - min_index
            val_i = self.support.meters_to_feet(data.ranges[i])

            # Object is symmetric if value mirrored is about equal to minimum distance
            if math.isnan(val_i) or abs(min_val - val_i) < LASER_SYMMETRIC_VALUE_THRESHOLD:
                # SYMMETTRIC
                self.symmetric_obstacle_detected = True
                self.asymmetric_obstacle_detected = False
            # Object is asymmetric if value mirrored is not equal to minimum distance
            else:
                # ASSYMETRIC
                self.symmetric_obstacle_detected = False
                self.asymmetric_obstacle_detected = True
            
            self.laser_data = data
            self.laser_min_index = min_index
        # Object not detected
        else:
            self.symmetric_obstacle_detected = False
            self.asymmetric_obstacle_detected = False

# Support class contains helper functions for other classes to utilize
class Support():

    # Convert meters to feet
    def meters_to_feet(self, val):
        return val * 3.28

    # Convert feet to meters
    def feet_to_meters(self, val):
        return val / 3.28

    # Convert radians to degrees
    def rad_to_deg(self, rad):
        return rad * 180 / math.pi

    # Return a random number in a range
    def get_random_number(self, min, max):
        return random.uniform(min, max)

# Class publishes velocity messages 
# as well as contain routine to be performed by robot
class Movement():

    def __init__(self, keyboard, laser):
        self.keyboard = keyboard
        self.laser = laser
        self.support = Support()

        self.velocity_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size = 10)

    # Move robot with message
    def velocity_publish(self, vel_msg):
        self.velocity_pub.publish(vel_msg)

    # Performs forward movement and random turning
    def autonomous(self):
        
        # Moving forward
        move_msg = Twist()
        move_msg.linear.x = self.support.feet_to_meters(LINEAR_SPEED_DEFAULT)

        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        print('Moving forward 1ft')
        
        while (current_distance < AUTONOMOUS_FORWARD_DISTANCE):
            if self.keyboard.is_teleop_controlled or self.laser.asymmetric_obstacle_detected or self.laser.symmetric_obstacle_detected:
                return
        
            self.velocity_publish(move_msg)
            t1 = rospy.Time.now().to_sec()
            current_distance = LINEAR_SPEED_DEFAULT * (t1 - t0)
        
        # Rotating
        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        target_angle = self.support.get_random_number(-15.0, 15.0)
        
        print('Rotating ' + str(round(target_angle, 2)))

        # Determine the direction to turn
        turn_msg = Twist()
        if target_angle >= 0:
            turn_msg.angular.z = ANGULAR_SPEED_DEFAULT
        else:
            turn_msg.angular.z = -ANGULAR_SPEED_DEFAULT


        while (current_angle < abs(target_angle)):
            if self.keyboard.is_teleop_controlled or self.laser.asymmetric_obstacle_detected or self.laser.symmetric_obstacle_detected:
                return
            
            self.velocity_publish(turn_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = self.support.rad_to_deg(ANGULAR_SPEED_DEFAULT) * (t1 - t0)        

    # Turn 180 degrees
    def escape(self):
        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        target_angle = 180

        turn_msg = Twist()
        turn_msg.angular.z = ANGULAR_SPEED_DEFAULT

        while (current_angle < abs(target_angle)):
            if self.keyboard.is_teleop_controlled:
               return

            self.velocity_publish(turn_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = self.support.rad_to_deg(ANGULAR_SPEED_DEFAULT) * (t1 - t0)

    # Turn until asymmetric obstacle is not visible
    def avoid(self):

        turn_msg = Twist()

        # Turn in the direction away from the closest obstacle
        if self.laser.laser_min_index < (len(self.laser.laser_data.ranges) - 1) / 2:
            turn_msg.angular.z = ANGULAR_SPEED_DEFAULT
        else:
            turn_msg.angular.z = -ANGULAR_SPEED_DEFAULT
        
        while self.laser.asymmetric_obstacle_detected:
            if self.keyboard.is_teleop_controlled or self.laser.symmetric_obstacle_detected:
                return
            
            self.velocity_publish(turn_msg)


def init_control_node():
    rospy.init_node('control_node', anonymous = False)
    rate = rospy.Rate(10)

    # Initialize all publishers and subscribers
    keyboard = Keyboard()
    bumper = Bumper()
    laser = Laser()
    
    movement = Movement(keyboard, laser)

    while not rospy.is_shutdown():

        # Halt if bumper collision detected
        if bumper.collision_detected:
            print('\rCOLLISION DETECTED: STOPPING...')
            vel_msg = Twist()
            movement.velocity_publish(vel_msg)
        # User teleop controlling
        elif keyboard.is_teleop_controlled:
            print('\rTeleop control detected')
            movement.velocity_publish(keyboard.vel_msg)
        # Symmetric obstacle found
        elif laser.symmetric_obstacle_detected:
            print('\rEncountered symmetric obstacle')
            movement.escape()
        # Asymmetric obstacle found
        elif laser.asymmetric_obstacle_detected:
            print('\rEncountered asymmetric obstacle')
            movement.avoid()
        # Auto-move
        else:
            movement.autonomous()
        
        rate.sleep()

if __name__ == '__main__':
    try:
        init_control_node()
    except rospy.ROSInterruptException:
        pass
