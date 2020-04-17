#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import numpy
from math import sin, pi
from sensor_msgs.msg import Imu, MagneticField
from pyquaternion import Quaternion
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import *

def calculate_goal(target_coordinate):
    goal = PoseStamped()
    target_orientation_quat = Quaternion(
        target_coordinate.pose.orientation.w,
        target_coordinate.pose.orientation.x,
        target_coordinate.pose.orientation.y,
        target_coordinate.pose.orientation.z
    )
    #a = my_quaternion.elements
    opposite_orientation = target_orientation_quat.conjugate
    individual_elements = opposite_orientation.elements
    obj_to_goal = target_orientation_quat.rotate((0.2, 0, 0)) # Returns a tuple
    goal.pose.position.x = target_coordinate.pose.position.x + obj_to_goal[0]
    goal.pose.position.y = target_coordinate.pose.position.y + obj_to_goal[1]
    goal.pose.position.z = target_coordinate.pose.position.z + obj_to_goal[2]
    goal.pose.orientation.w = individual_elements[0]
    goal.pose.orientation.x = individual_elements[1]
    goal.pose.orientation.y = individual_elements[2]
    goal.pose.orientation.z = individual_elements[3]
    h = std_msgs.msg.Header()
    h.stamp = rospy.get_rostime()
    h.frame_id = "map"
    # # h.seq = 2
    goal.header = h
    return goal
    


def get_map():
    rospy.wait_for_service('/static_map')

    try:
        get_occupancy_map = rospy.ServiceProxy('/static_map', GetMap)
        occupancy_map = get_occupancy_map()
        return occupancy_map
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == '__main__':
    try:
        
        rospy.init_node('goal_publisher_node',anonymous=False)
        rospy.sleep(0.2) 
        rospy.loginfo("goal publisher has started")
        goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)

        # occupancy_map = get_map()
        while not rospy.is_shutdown():
            object_location = PoseStamped()
            object_location.pose.position.x = 1
            object_location.pose.position.y = 1
            object_location.pose.position.z = 0
            object_location.pose.orientation.x = 0
            object_location.pose.orientation.y = 0
            object_location.pose.orientation.z = 1
            object_location.pose.orientation.w = 0

            target_coordinate = object_location
            goal = calculate_goal(target_coordinate)
            goal_pub.publish(goal)
            rospy.loginfo((goal))
            rospy.loginfo("goal published successfully")
            
            rospy.sleep(0.2)


    
    except rospy.ROSInterruptException:
        port.close()
    
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down paro_depth node...")
