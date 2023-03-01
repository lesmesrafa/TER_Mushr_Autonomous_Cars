#!/usr/bin/env python3


""" 
 \ brief     Time elastic band controller ROS node (time_elastic_band_controller)
 \ author    Rafael Martin Lesmes and Eduardo Freyre Gomez
 \ summary   In this document is implemented the rosnode which allows the Mushr car to create the path to follow detecting obstacles.
"""

# ROS dependecies
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion

# Python dependencies
import math
import rospy
import numpy as np


class TimeElasticBandController:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('time_elastic_band_controller')
        self.min_threshold = 0.1
        self.max_scope_laser = 9.0
        self.speed = float(rospy.get_param("~speed"))
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
        self.old_target_pos = None
        # Obstacles
        self.obstacles = None
        self.min_dist_object = 1.0
        
        
        self.obstacles_front = None
        self.obstacles_right = None
        self.obstacles_left = None
        
        
        
        # self.flag = False
        self.path_saved = []
   
        # Publishers
        self.publish_path = rospy.Publisher("/path_to_follow", Path, queue_size=1)
        # Subscribers 
        self.sub_car_pose = rospy.Subscriber('/car/car_pose', PoseStamped, self.odom_callback)
        self.sub_car_scan = rospy.Subscriber('/car/scan', LaserScan, self.laser_callback)
        self.sub_goal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
    
    
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

        
    def goal_callback(self, msg):
        """Save desired destination."""
        self.target_pos = (round(msg.pose.position.x, 1), round(msg.pose.position.y, 1))
       
       
    ##############################################################################
    ############################ Localize obstacles ##############################
    ##############################################################################
    
    
    def calculate_coordinates(self, index, reading):
        """Convert laser reading index and distance to map coordinates."""
        # Calculate map coordinates using trigonometry and current position of the robot
        angle = self.laser_start_angle + self.laser_angle_increment * index + self.yaw
        x = reading * math.cos(angle) + self.current_pos[0]
        y = reading * math.sin(angle) + self.current_pos[1]

        return x, y


    def get_obstacles(self):
        """Update map data with current laser readings.""" 
        # Checking if laser scan data is not None
        if self.laser_readings is None:
            rospy.logwarn("Laser scan is not yet initialized, skipping updating map.")
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
    ############################# TEB Controller Algorithm #######################
    ##############################################################################


    def min_distance(self, points1, points2):
        """Returns the minimum distance between any point in points1 and any point in points2."""
        min_dist = math.inf
        for p1 in points1:
            for p2 in points2:
                dist = ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**0.5
                if dist < min_dist:
                    min_dist = dist
        return min_dist
    
    
    def get_neighbors(self, point):
            """Calculate neighbors points of 'point'.""" 
            x, y = point
            growing_factor = 0.1
            neighbors = [(round(x-growing_factor,1), round(y-growing_factor,1)), (round(x-growing_factor,1), y), (round(x-growing_factor,1), round(y+growing_factor,1)), (x, round(y-growing_factor,1)), (x, round(y+growing_factor,1)), (round(x+growing_factor,1), round(y-growing_factor,1)), (round(x+growing_factor,1), y), (round(x+growing_factor,1), round(y+growing_factor,1))]
            return neighbors


    def calculate_cost(self, point, target):
            """Calculate cost to go from 'point' to target.""" 
            if not isinstance(target, list):
                return abs(np.sqrt((target[0]-point[0])**2 + (target[1]-point[1])**2))
            

    def check_distance(self, point, points):
        """Calculate distance from 'point' to the closest point of 'points' list."""
        a, b = point
        min_dist = 999999
        for p in points:
            dist = math.sqrt((p[0] - a)**2 + (p[1] - b)**2)
            if dist < min_dist:
                min_dist = dist
        return min_dist
       
       
    def rdp(self, path, epsilon):
        """
        Simplifies a path using the Ramer-Douglas-Peucker algorithm.
        """
        def point_line_distance(point, start, end):
            """
            Calculates the distance between a point and a line segment.
            """
            if np.all(np.equal(start, end)):
                return np.linalg.norm(np.array(point) - np.array(start))

            l2 = np.linalg.norm(np.array(end) - np.array(start)) ** 2
            t = max(0, min(1, np.dot(np.array(point) - np.array(start), np.array(end) - np.array(start)) / l2))
            projection = np.array(start) + t * (np.array(end) - np.array(start))
            return np.linalg.norm(np.array(point) - projection)
        dmax = 0
        index = 0
        for i in range(1, len(path) - 1):
            d = point_line_distance(path[i], path[0], path[-1])
            if d > dmax:
                index = i
                dmax = d
        if dmax > epsilon:
            results = self.rdp(path[:index + 1], epsilon)[:-1] + self.rdp(path[index:], epsilon)
        else:
            results = [path[0], path[-1]]

        return results
    
    
    def smooth_path(self, path, min_distance=1):
        """
        Smooth path generated by Ramer-Douglas-Peucker algorithm.
        """
        new_path = [path[0]] # initialize new path with first point
    
        for i in range(1, len(path)-1):
            x0, y0 = path[i-1]
            x1, y1 = path[i]
            x2, y2 = path[i+1]
            dist = math.hypot(x2-x0, y2-y0) # distance between i-1 and i+1
            
            if dist > min_distance:
                new_path.append(path[i])
            else:
                dist1 = math.hypot(x1-x0, y1-y0) # distance between i-1 and i
                dist2 = math.hypot(x2-x1, y2-y1) # distance between i and i+1
                dist3 = math.hypot(x2-x0, y2-y0) # distance between i-1 and i+1
                
                if dist1 <= dist2 and dist1 <= dist3:
                    new_path[-1] = path[i] # replace last point with current point
                elif dist2 <= dist3:
                    pass # don't add current point to new path
                else:
                    new_path.append(path[i+1]) # add next point to new path
                
        new_path.append(path[-1]) # add last point to new path
        return new_path
    
        
    def teb_path_generation(self):
        """Calculate shortest path to target.""" 
        # Check if odometry is not None
        if self.xy_pos is None: 
            rospy.logwarn("Odometry is not yet initialized, skipping path creation.")
            return
        # Check if target is not None
        if self.target_pos is None: 
            rospy.logwarn("Target point is not yet initialized, path creation.")
            return
        # Check if laser is not None
        if self.obstacles is None: 
            rospy.logwarn("Laser is not yet initialized, path creation.")
            return
        # Initialize start, end and obstacles
        start = (self.xy_pos[0] + 1, self.xy_pos[1] + 1)
        end = self.target_pos
        obstacles = self.obstacles
        
        visited = {start:self.calculate_cost(start, end)}
      
        path = [start]
       
        current_node = None
        # While the visited list is not empty
        while len(visited) != 0:
            if len(path) == 1:
                current_node = start
            # Get the node with the minimum cost
            current_cost = visited[current_node]
            # If the cost is below a threshold, return the path
            if current_cost <= 0.1 or len(path) == 400*self.speed:
                self.path_saved = path
                return path
            # Get the neighbors of the current node and their costs
            neighbors = self.get_neighbors(current_node)
            set_neighbors = {}
            # For each neighbor, calculate its cost and add it to a set
            for neighbor in neighbors:
                if neighbor in obstacles or neighbor in visited.keys():
                    continue
                if len(obstacles) > 5:
                    if self.check_distance(neighbor, obstacles) <= self.min_dist_object:
                        
                        continue

                set_neighbors[neighbor] = self.calculate_cost(neighbor, end)
            # Get the neighbor with the minimum cost and add it to the path 
            if len(set_neighbors) == 0:
                break
            else:
                current_node = min(set_neighbors, key=set_neighbors.get)
                visited[current_node] = set_neighbors[current_node]
                path.append(current_node)

    ##############################################################################
    ################################ Run the node ################################
    ##############################################################################

    def run(self):
        """Main loop."""
        rate = rospy.Rate(10)
        # Continuously update the map and calculate the path
        cont = 0 
        while not rospy.is_shutdown():
            self.get_obstacles()
            path = self.teb_path_generation()
           
            # If a path is found, publish it as a path message
            if path is not None:
                if self.target_pos != self.old_target_pos or cont == 0:
                    self.old_target_pos = self.target_pos 
                    cont = 0
                    

                if cont == 0:
                    cont += 1
                    path = path#self.smooth_path(self.rdp(path, epsilon=200.0))
                    path_msg = Path()
                    path_msg.header.frame_id = "map"
                    for i, (x, y) in enumerate(path):
                        pose = PoseStamped()
                        pose.header.frame_id = "map"
                        pose.header.seq = i
                        pose.pose.position.x = x
                        pose.pose.position.y = y
                        path_msg.poses.append(pose)
                    self.publish_path.publish(path_msg)
                    
                else:
                    path_msg = Path()
                    path_msg.header.frame_id = "map"
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