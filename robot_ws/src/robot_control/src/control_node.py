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


# vel_msg
# is_teleop_controlled
class Keyboard():

    def __init__(self):
        self.support = Support()

        self.is_teleop_controlled = False
        self.vel_msg = Twist()

        self.keyboard_sub = rospy.Subscriber('/robot/keyboard_input', keyboard, self.keyboard_callback)

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
        self.laser_data = LaserScan()
        self.laser_min_index = 0

        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)

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
            ranges.append(self.support.meters_to_feet(data.ranges[i]))
        
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
            if self.keyboard.is_teleop_controlled or self.laser.asymmetric_obstacle_detected or self.laser.symmetric_obstacle_detected:
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

        keyboard = Keyboard()
        bumper = Bumper()
        laser = Laser()
        
        movement = Movement(keyboard, laser)

        while not rospy.is_shutdown():
            if bumper.collision_detected:
                print('COLLISION DETECTED: STOPPING...')
                vel_msg = Twist()
                movement.velocity_publish(vel_msg)
            
            elif keyboard.is_teleop_controlled:
                print('Teleop control detected')
                movement.velocity_publish(keyboard.vel_msg)
            elif laser.symmetric_obstacle_detected:
                print('Encountered symmetric obstacle')
                movement.escape()
            elif laser.asymmetric_obstacle_detected:
                print('Encountered asymmetric obstacle')
                movement.avoid()
            else:
                movement.autonomous()
                #print('its fine')
            
            rate.sleep()
        

    except rospy.ROSInterruptException:
        pass
