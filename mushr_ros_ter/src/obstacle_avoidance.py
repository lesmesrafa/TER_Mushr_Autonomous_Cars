#!/usr/bin/env python3

""" 
 \ brief     Obstacle avoidance node
 \ author    Rafael Martin Lesmes
 \ summary   
"""

# Python dependencies
import math
from statistics import mean
# ROS dependecies
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from geometry_msgs.msg import Point, Pose, PoseWithCovariance, PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler

def bsic_callback(msg):
    """
    This function implements the Braitemberg Controller algorithm.

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

    
     # Go straight
    if (regions["straight"] >= threshold) or (regions["right"] < threshold and regions["left"] < threshold):
        info = ">-- Go straight --<"
        rospy.loginfo(">>> GO STRAIGHT")
        drive = AckermannDrive(speed=velocity, steering_angle=0.0) 
    # Go right  
    elif (regions["straight"] < threshold) and (regions["left"] < threshold) and (regions["right"] >= threshold):
        info = ">-- Go right --<"
        drive = AckermannDrive(speed=velocity, steering_angle=-yaw) 
    # Go left
    elif (regions["straight"] < threshold) and (regions["right"] < threshold) and (regions["left"] >= threshold):
        info = ">-- Go left --<"
        drive = AckermannDrive(speed=velocity, steering_angle=yaw) 
    # Go left or right
    elif (regions["straight"] < threshold) and (regions["left"] >= threshold) and (regions["right"] >= threshold):
        if regions["left"] > regions["right"]:
            info = ">-- Go left --<"
            drive = AckermannDrive(speed=velocity, steering_angle=yaw) 
        else: 
            info = ">-- Go right --<"
            drive = AckermannDrive(speed=velocity, steering_angle=-yaw) 
            
    rospy.loginfo(info)    
    pub_controls.publish(AckermannDriveStamped(drive=drive))
   

    
def bsic_ini():
    
    global pub_controls, velocity, yaw, threshold
    
    rospy.init_node('obstacle_avoidance', anonymous=True)
    
    lidar_topic = rospy.get_param("~lidar_topic")
    control_topic = rospy.get_param("~control_topic")
    velocity = float(rospy.get_param("~velocity"))
    yaw = float(rospy.get_param("~yaw"))
    threshold = float(rospy.get_param("~threshold"))
    
    rospy.Subscriber(lidar_topic, LaserScan, bsic_callback)
    pub_controls = rospy.Publisher(control_topic, AckermannDriveStamped, queue_size=1)
    
    rospy.spin()
    
if __name__ == "__main__":
    bsic_ini()