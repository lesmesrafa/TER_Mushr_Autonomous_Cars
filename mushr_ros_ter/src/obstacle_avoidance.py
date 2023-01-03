#!/usr/bin/env python3

""" 
 \ brief     Obstacle avoidance node
 \ author    Rafael Martin Lesmes
 \ summary   In this document is implemented the rosnode which allows the Mushr car to avoid obstacles by following a Braitemberg approach.
"""

# Python dependencies
import math
from statistics import mean
# ROS dependecies
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped


def lidar_callback(msg):
    """
    This function implements the main algorithm that makes the car avoid obstacles.

    Args:
        msg: data read from LIDAR sensor  
    """

    # Values < range_min or > range_max should be discarded (10.0 meters).
    # Nan values = max possible value (10.0 meters).
    laser_distances = [round(distance, 2) if not math.isnan(distance) and (msg.range_min <= distance) and (msg.range_max >= distance) else 10.0 for distance in msg.ranges]
    # Defining which parts of the LIDAR corresponds to a movement.
    regions = {
        "right": mean(laser_distances[99:279]), 
        "straight": mean(laser_distances[279:459]),
        "left": mean(laser_distances[459:619]) 
        }

    condition_left = regions["left"] <= THRESHOLD / 4.0
    condition_right = regions["right"] <= THRESHOLD / 4.0
    condition_straight = regions["straight"] <= THRESHOLD

    drive = None
     # Go straight
    if not condition_straight or (not condition_straight and condition_right and condition_left):
        action_info = "Go straight"
        drive = AckermannDrive(speed=VELOCITY, steering_angle=0.0) 
    # Go right  
    elif condition_straight and condition_left and not condition_right:
        action_info = "Go right"
        drive = AckermannDrive(speed=VELOCITY, steering_angle=-YAW) 
    # Go left
    elif condition_straight and condition_right and not condition_left:
        action_info = "Go left"
        drive = AckermannDrive(speed=VELOCITY, steering_angle=YAW) 
    # Go left or right
    elif condition_straight and not condition_left and not condition_right:
        if regions["left"] > regions["right"]:
            action_info = "Go left"
            drive = AckermannDrive(speed=VELOCITY, steering_angle=YAW) 
        else: 
            action_info = "Go right"
            drive = AckermannDrive(speed=VELOCITY, steering_angle=-YAW) 
    # Stop
    else:
        action_info = "Stop"
        drive = AckermannDrive(speed=0.0, steering_angle=0.0) 

            
    rospy.loginfo("Action == " + action_info)    
    pub_controls.publish(AckermannDriveStamped(drive=drive))
   

def main():
    
    global pub_controls, VELOCITY, YAW, THRESHOLD
    # Node name
    rospy.init_node('obstacle_avoidance', anonymous=True)
    # Environment variables
    lidar_topic = rospy.get_param("~lidar_topic")
    control_topic = rospy.get_param("~control_topic")
    VELOCITY = float(rospy.get_param("~velocity"))
    YAW = float(rospy.get_param("~yaw"))
    THRESHOLD = float(rospy.get_param("~threshold"))
    # Subscriber/publisher calls
    rospy.Subscriber(lidar_topic, LaserScan, lidar_callback)
    pub_controls = rospy.Publisher(control_topic, AckermannDriveStamped, queue_size=1)


    rospy.spin()
    
if __name__ == "__main__":
    main()