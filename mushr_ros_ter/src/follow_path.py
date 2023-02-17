#!/usr/bin/env python3


""" 
 \ brief     Follow path ROS node (follow_path_node)
 \ author    Rafael Martin Lesmes and Eduardo Freyre Gomez
 \ summary   In this document is implemented the rosnode which allows the Mushr car to follow a path.
"""

# ROS dependecies
import rospy
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

# Python dependencies
import math
import numpy as np
from statistics import mean

# Global variables
path = None
current_pose = None
angle_tolerance = 0.1 # Angle tolerance to consider the object as facing the target
laser_distances = None


def laser_callback(msg):
    """Save current laser readings."""  
    global laser_distances, regions, condition_left, condition_right, condition_front, condition_two_front_regions, front_left, front_right
    # Values < range_min or > range_max should be discarded (10.0 meters).
    # Nan values = max possible value (10.0 meters).
    laser_distances = [round(distance, 2) if not math.isnan(distance) and (msg.range_min <= distance) and (msg.range_max >= distance) else 10.0 for distance in msg.ranges]
    # Defining which parts of the LIDAR corresponds to a movement.
    regions = {
        "right": mean(laser_distances[99:279]), 
        "front": mean(laser_distances[279:459]),
        "left": mean(laser_distances[459:619]) 
        }
    # Define a threshold value for detecting obstacles
    THRESHOLD = 7.0
    condition_left = regions["left"] <= THRESHOLD
    condition_right = regions["right"] <= THRESHOLD
    condition_front = regions["front"] <= THRESHOLD
    condition_two_front_regions = abs(mean(laser_distances[279:459][:80]) - mean(laser_distances[279:459][80:])) > 0.01 
    
    front_left = mean(laser_distances[279:459][80:])
    front_right = mean(laser_distances[279:459][:80])


def path_callback(msg):
    """Get the coordinates of the path."""
    global path
    # Initializing an empty list to store path coordinates
    path = []
    # Looping through the path coordinates and appending them to the list
    for i in range(len(msg.poses)):
        x = msg.poses[i].pose.position.x
        y = msg.poses[i].pose.position.y
        path.append((x, y))
   

def current_pose_callback(msg):
    """Get the coordinates of the car."""
    global current_pose
    current_pose = msg
    
    
def follow_path_logic(drive_publisher):
    """Takes a drive_publisher argument and follows the given path."""
    # Loop through each position in the path
    for pos in path:
        # Check if the distance to the target is greater than 0.1 units
        if math.sqrt((pos[0] - current_pose.pose.position.x)**2 + (pos[1] - current_pose.pose.position.y)**2) > 0.1:
            # Calculate the distance to the target
            distance_to_object = math.sqrt((pos[0]- current_pose.pose.position.x)**2 + (pos[1] - current_pose.pose.position.y)**2)
            # Calculate the angle to the target
            relative_angle = math.atan2(pos[1] - current_pose.pose.position.y, pos[0] - current_pose.pose.position.x)        
            # Get the current yaw angle of the car
            yaw_car = euler_from_quaternion([current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w])[2]
            # Calculate the difference between the target angle and the car's yaw angle
            angle_diff = relative_angle - yaw_car
            # Wrap the angle difference to [-pi, pi] range
            angle_diff = math.atan2(np.sin(angle_diff), np.cos(angle_diff))
            # If we are within 0.3 units of the target or we have reached the last point on the path      
            if abs(distance_to_object) <= 0.3 or len(path) <= 1.0:
                # Set the drive speed to 0 and the steering angle to 0
                drive = AckermannDrive(speed=0.0, steering_angle=0.0)  
            else:
                # If the angle difference is less than -0.1, turn left
                if angle_diff < -0.1:
                    drive = AckermannDrive(speed=SPEED, steering_angle=-0.3)
                # If the angle difference is greater than 0.1, turn right
                elif angle_diff > 0.1:
                    drive = AckermannDrive(speed=SPEED, steering_angle=0.3)
                # Otherwise, drive straight
                else:
                    drive = AckermannDrive(speed=SPEED, steering_angle=0.0)
             # Publish the drive command to the drive publisher   
            drive_publisher.publish(AckermannDriveStamped(drive=drive))


def follow_path():
    """Run all the logic to follow the path and to avoid obstacles."""
    global path, current_pose, angle_tolerance, regions, condition_left, condition_right, condition_front, condition_two_front_regions, front_left, front_right, laser_distances
    # Define a publisher for the drive commands
    drive_publisher = rospy.Publisher('/car/mux/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=1)
    # Loop continuously while the ROS node is running
    while not rospy.is_shutdown():
        # Check if all required information is available   
        if path is not None and current_pose is not None and laser_distances is not None:
            # Calculate the relative angle of the car to the final point of the path
            relative_angle = math.atan2(path[-1][1] - current_pose.pose.position.y, path[-1][0] - current_pose.pose.position.x)        
            # Get the current yaw angle of the car
            yaw_car = euler_from_quaternion([current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w])[2]
            # Calculate the difference between the relative angle and the yaw angle
            angle_diff = relative_angle - yaw_car
            angle_diff = math.atan2(np.sin(angle_diff), np.cos(angle_diff))
            # Check if there are no obstacles or only obstacles on one side
            if (not condition_front and not condition_right and not condition_left) or (not condition_front and condition_right and condition_left) or (not condition_front and condition_left and angle_diff < 0.4) or (not condition_front and condition_right and angle_diff > -0.4):
                # Follow the path while avoiding obstacles
                follow_path_logic(drive_publisher)     
            else:
                # Avoid obstacles
                drive = None
                # Go straight if there are no obstacles in front or if there are only obstacles on the sides
                if not condition_front or (not condition_front and condition_right and condition_left):
                    drive = AckermannDrive(speed=SPEED, steering_angle=0.0)  
                # Turn right if there are obstacles on the left 
                elif condition_front and condition_left and not condition_right:
                    drive = AckermannDrive(speed=SPEED, steering_angle=-0.3) 
                # Turn left if there are obstacles on the right
                elif condition_front and condition_right and not condition_left:
                    drive = AckermannDrive(speed=SPEED, steering_angle=0.3) 
                # Turn left or right if there are no obstacles in front but obstacles on both sides
                elif condition_front and not condition_left and not condition_right:
                    if regions["left"] > regions["right"]:
                        drive = AckermannDrive(speed=SPEED, steering_angle=0.3) # Go left
                    else: 
                        drive = AckermannDrive(speed=SPEED, steering_angle=-0.3) # Go right
                # Turn left or right if there are multiple obstacles in front      
                elif condition_two_front_regions:
                    if front_left > front_right:
                        drive = AckermannDrive(speed=SPEED, steering_angle=0.3) # Go left
                    else: 
                        drive = AckermannDrive(speed=SPEED, steering_angle=-0.3) # Go right
                # Stop if there are obstacles
                else:
                    drive = AckermannDrive(speed=0.0, steering_angle=0.0) 

                drive_publisher.publish(AckermannDriveStamped(drive=drive))
                


if __name__ == '__main__':
    # Initializes a ROS node with the name 'follow_path_node'
    rospy.init_node('follow_path_node')
    # Subscribes to the topics 'path_to_follow', 'car_pose', and 'car/scan'
    rospy.Subscriber('/path_to_follow', Path, path_callback)
    rospy.Subscriber('/car/car_pose', PoseStamped, current_pose_callback)
    rospy.Subscriber('/car/scan', LaserScan, laser_callback)
    # Retrieves the 'speed' parameter from the ROS parameter server
    SPEED = float(rospy.get_param("~speed"))
    # Calls the 'follow_path' function
    follow_path()
    # Enters an infinite loop, allowing ROS to continue running until the node is stopped
    rospy.spin()