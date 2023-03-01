#!/usr/bin/env python3


""" 
 \ brief     Follow path ROS node (follow_path_node)
 \ author    Rafael Martin Lesmes and Eduardo Freyre Gomez
 \ summary   In this document is implemented the rosnode which allows the Mushr car to follow a path.
"""

# ROS dependecies
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

# Python dependencies
import math
import numpy as np

# Global variables
path = None
current_pose = None


def goal_callback(msg):
    global target_pos
    """Save desired destination."""
    target_pos = (round(msg.pose.position.x, 1), round(msg.pose.position.y, 1))


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


def follow_path():
    """Run all the logic to follow the path and to avoid obstacles."""
    global path, current_pose
    # Define a publisher for the drive commands
    drive_publisher = rospy.Publisher('/car/mux/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=1)
    # Loop continuously while the ROS node is running
    while not rospy.is_shutdown():
        # Check if all required information is available   
        if path is not None and current_pose is not None and len(path) > 0:
            # Calculate the relative angle of the car to the final point of the path
            relative_angle = math.atan2(path[-1][1] - current_pose.pose.position.y, path[-1][0] - current_pose.pose.position.x)        
            # Get the current yaw angle of the car
            yaw_car = euler_from_quaternion([current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w])[2]
            # Calculate the difference between the relative angle and the yaw angle
            angle_diff = relative_angle - yaw_car
            angle_diff = math.atan2(np.sin(angle_diff), np.cos(angle_diff))
            # Loop through each position in the path
            for pos in path[1:]:   
                distance_to_target = math.sqrt((target_pos[0]- current_pose.pose.position.x)**2 + (target_pos[1] - current_pose.pose.position.y)**2)
                # Calculate the angle to the target
                relative_angle = math.atan2(pos[1] - current_pose.pose.position.y, pos[0] - current_pose.pose.position.x)        
                # Get the current yaw angle of the car
                yaw_car = euler_from_quaternion([current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w])[2]
                # Calculate the difference between the target angle and the car's yaw angle
                angle_diff = relative_angle - yaw_car
                # Wrap the angle difference to [-pi, pi] range
                angle_diff = math.atan2(np.sin(angle_diff), np.cos(angle_diff))
                # If we are within 0.3 units of the target or we have reached the last point on the path      
                if abs(distance_to_target) <= 0.3 or pos is path[-1]:
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
                


if __name__ == '__main__':
    # Initializes a ROS node with the name 'follow_path_node'
    rospy.init_node('follow_path_node')
    # Subscribes to the topics 'path_to_follow', 'car_pose', and '/move_base_simple/goal'
    rospy.Subscriber('/path_to_follow', Path, path_callback)
    rospy.Subscriber('/car/car_pose', PoseStamped, current_pose_callback)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback)
    # Retrieves the 'speed' parameter from the ROS parameter server
    SPEED = float(rospy.get_param("~speed"))
    # Calls the 'follow_path' function
    follow_path()
    # Enters an infinite loop, allowing ROS to continue running until the node is stopped
    rospy.spin()