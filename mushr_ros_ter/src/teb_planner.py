#!/usr/bin/env python3


""" 
 \ brief     Time elastic band controller ROS node (time_elastic_band_controller)
 \ author    Rafael Martin Lesmes and Eduardo Freyre Gomez
 \ summary   In this document is implemented the rosnode which allows the Mushr car to create the path to follow detecting obstacles.
"""

# ROS dependecies
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from tf.transformations import euler_from_quaternion

# Python dependencies
import math
import rospy
import numpy as np


class TimeElasticBandController:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('time_elastic_band_controller')
        # Set up map and occupancy grid
        self.origin = None
        self.width = None
        self.height = None
        self.resolution = None
        # Laser configuration
        self.laser_start_angle = None
        self.laser_angle_increment = None
        self.laser_readings = None
        # Set up current position
        self.current_pos = None
        self.xy_pos = None
        self.yaw = None
        # Set up desired destination
        self.target_pos = None
        # Obstacles
        self.obstacles = None

   
        # Publishers
        self.publish_path = rospy.Publisher("/path_to_follow", Path, queue_size=1)
        # Subscribers 
        self.sub_car_pose = rospy.Subscriber('/car/car_pose', PoseStamped, self.odom_callback)
        self.sub_car_scan = rospy.Subscriber('/car/scan', LaserScan, self.laser_callback)
        self.sub_goal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
    
    
    ##############################################################################
    ################################# Callbacks ##################################
    ##############################################################################
    
    
    def quaternion_to_yaw(self, quaternion):
        """Convert quaternion to yaw angle."""
        # Extracting the quaternion values as a list
        orientation_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        # Converting quaternion to Euler angles (roll, pitch, yaw)
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        return yaw 
    
    
    def odom_callback(self, msg):
        """Save current position and orientation."""
        # Saving the current position (x, y) and yaw angle
        self.current_pos = (round(msg.pose.position.x, 2), round(msg.pose.position.y, 2), self.quaternion_to_yaw(msg.pose.orientation))
        # Saving the current position (x, y) only
        self.xy_pos = (round(msg.pose.position.x, 2), round(msg.pose.position.y, 2))
        # Saving the current yaw angle only
        self.yaw = self.quaternion_to_yaw(msg.pose.orientation)
        
        
    def laser_callback(self, msg):
        """Save current laser readings and update map."""
        # Update laser readings while discarding out-of-range readings and NaN values        
        self.laser_readings = [round(distance, 2) if not math.isnan(distance) and (msg.range_min <= distance) and (msg.range_max >= distance) else msg.range_max for distance in list(msg.ranges)]
        # Discard readings with too much difference to adjacent readings
        for i in range(len(self.laser_readings)):
            if i == 0 and abs(msg.ranges[i] - msg.ranges[i+1]) > 0.1:
                self.laser_readings[i] = 10.0
            elif i == 719 and abs(msg.ranges[i] - msg.ranges[i-1]) > 0.1:
                self.laser_readings[i] = 10.0
            elif abs(msg.ranges[i] - msg.ranges[i-1]) > 0.1 and abs(msg.ranges[i] - msg.ranges[i+1]) > 0.1:
                self.laser_readings[i] = 10.0
        # Save laser start angle and increment
        self.laser_start_angle = msg.angle_min
        self.laser_angle_increment = msg.angle_increment
        
    
    def map_callback(self, msg):
        """Save map data and metadata."""
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        
        
    def goal_callback(self, msg):
        """Save desired destination."""
        self.target_pos = (round(msg.pose.position.x, 1), round(msg.pose.position.y, 1))
       
       
    ##############################################################################
    ############################ Localize obstacles ##############################
    ##############################################################################
    
    
    def calculate_coordinates(self, index, reading):
        """Convert laser reading index and distance to map coordinates."""
        # Calculate map coordinates using trigonometry and current position of the robot
        angle = self.laser_start_angle + self.laser_angle_increment * index
        x = reading * math.cos(angle) + self.current_pos[0]
        y = reading * math.sin(angle) + self.current_pos[1]
        return x, y
    
    
    def update_map(self):
        """Update map data with current laser readings.""" 
        # Checking if laser scan data is not None
        if self.laser_readings is None:
            rospy.logwarn("Laser scan is not yet initialized, skipping updating map.")
            return
        # Checking if map data is not None
        if self.origin is None: 
            rospy.logwarn("Map is not yet initialized, skipping updating map.")
            return
        # Getting obstacles
        self.obstacles = []
        for i, reading in enumerate(self.laser_readings):
            if reading <= self.min_threshold or reading == float('inf') or reading == float('nan') or reading == 'nan' or reading > self.max_scope_laser:
                continue
            x, y = self.calculate_coordinates(i, reading)
            # Saving obstacles   
            self.obstacles.append((round(x, 1), round(y, 1)))       
        
    
    ##############################################################################
    ############################# D* Lite Algorithm ##############################
    ##############################################################################
    
    
    def get_neighbors(self, point):
        """Calculate neighbors of 'point'.""" 
        x, y = point
        return [(x-0.1, y-0.1), (x-0.1, y), (x-0.1, y+0.1), (x, y-0.1), (x, y+0.1), (x+0.1, y-0.1), (x+0.1, y), (x+0.1, y+0.1)]


    def calculate_cost(self, point, target):
        """Calculate cost to go from 'point' to target.""" 
        if not isinstance(target, list):
            return abs(np.sqrt((target[0]-point[0])**2 + (target[1]-point[1])**2))
       
        
    def d_star_lite(self):
        """Calculate shortest path to target impleemnting D* Lite algorithm.""" 
        # Check if odometry is not None
        if self.xy_pos is None: 
            rospy.logwarn("Odometry is not yet initialized, skipping D* lite.")
            return
        # Check if target is not None
        if self.target_pos is None: 
            rospy.logwarn("Target point is not yet initialized, skipping D* lite.")
            return
        # Check if laser is not None
        if self.obstacles is None: 
            rospy.logwarn("Laser is not yet initialized, skipping D* lite.")
            return
        # Initialize start, end and obstacles
        start = self.xy_pos
        end = self.target_pos
        obstacles = self.obstacles
            
        visited = {start:self.calculate_cost(start, end)}
        path = [start]
        # While the visited list is not empty
        while len(visited) != 0:
            # Get the node with the minimum cost
            current_node = min(visited, key=visited.get)
            current_cost = visited[current_node]
            # If the cost is below a threshold, return the path
            if current_cost < 0.1:
                return path
            # Remove the node from the visited list
            del visited[current_node]
            # Get the neighbors of the current node and their costs
            neighbors = self.get_neighbors(current_node)
            set_neighbors = {}
            # For each neighbor, calculate its cost and add it to a set
            for neighbor in neighbors:
                if neighbor in obstacles: 
                    continue
                set_neighbors[neighbor] = self.calculate_cost(neighbor, end)
            # Get the neighbor with the minimum cost and add it to the path 
            future_node = min(set_neighbors, key=set_neighbors.get)
            visited[future_node] = set_neighbors[future_node]
            path.append(future_node)
            
    
    ##############################################################################
    ################################ Run the node ################################
    ##############################################################################
    
    
    def run(self):
        """Main loop."""
        rate = rospy.Rate(10)
        # Continuously update the map and calculate the path
        while not rospy.is_shutdown():
            self.update_map()
            path = self.d_star_lite()
            # If a path is found, publish it as a path message
            if path is not None:
                path_msg = Path()
                path_msg.header.frame_id = "map"
                # Iterate through each point in the path, and append it to the path message
                for i, (x, y) in enumerate(path):
                    pose = PoseStamped()
                    pose.header.frame_id = "map"
                    pose.header.seq = i
                    pose.pose.position.x = x
                    pose.pose.position.y = y
                    path_msg.poses.append(pose)
                self.publish_path.publish(path_msg)
            rate.sleep()


if __name__ == '__main__':
    controller = TimeElasticBandController()
    controller.run()