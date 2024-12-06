import copy
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid
from random import uniform
from geometry_msgs.msg import Quaternion, Pose2D, Twist
import math
from example_interfaces.msg import Float32
from example_interfaces.msg import UInt8
import numpy as np

def quaternion_from_euler(yaw, pitch=0, roll=0):
    """
    Convert Euler angles to Quaternion.
    """
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def euler_from_quaternion(q):
    """
    Converts a quaternion to Euler angles (roll, pitch, yaw).
    Only yaw is relevant for 2D planar motion.
    """
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw

def classify_light_dark(value):
    """
    checks the sensor value as light or dark and calculates the probability.
    
    Args:
        value (int or float): The sensor value to classify.
    
    Returns:
        tuple: (Light/Dark, probability)
               Light/Dark: True (light) or False (dark)
               probability: Probability of the classification.
    """
    #ranges for light, dark, overlap (min, max)
    light_range = (101, 135)
    dark_range = (122, 155)
    overlap = (dark_range[0], light_range[1]) #makes more sense visually on a line chart
    

    if light_range[0] <= value <= light_range[1] and dark_range[0] <= value <= dark_range[1]:
        #within the overlapping region, compute probabilities by comparing to which edge its closer to
        light_prob = (light_range[1] - value) / (light_range[1] - overlap[0])
        dark_prob = (value - dark_range[0]) / (overlap[1] - dark_range[0])
        if light_prob > dark_prob:
            return True, light_prob
        else:
            return False, dark_prob

    #If not in overlap, classify based on ranges
    elif light_range[0] <= value <= light_range[1]:
        return True, 0.8
    elif dark_range[0] <= value <= dark_range[1]:
        return False, 0.8
    else:
        return "unknown", 0.0  #NaN and whatnot

def is_particle_on_light_spot(particle, map_data, resolution, width, height):
    """
    Determines if each particle is on a light or dark spot.

    Args:
        particle 
        map_data (list): The occupancy grid as a 1D list in reverse order.
        resolution (float): The resolution of the map (size of each grid cell in world units).
        width (int): The width of the grid in cells.
        height (int): The height of the grid in cells.

    Returns:
        True/False for Light/Dark
    """
    
    left, bottom, right, top = 0, 0, width * resolution, height * resolution
    particle_x, particle_y = particle.pose.position.x, particle.pose.position.y

    # In bounds
    if particle_x >= left and particle_x <= right and particle_y >= bottom and particle_y <= top:       
        # Convert world coordinates to grid indices
        grid_x = int(particle_x / resolution)
        grid_y = int(particle_y / resolution)

        # Compute the 1D index for the map data
        index = grid_y * width + grid_x

        # Check the occupancy value
        occupancy_value = map_data[index]
        if(occupancy_value == 0):
            return True #light 0
        else:
            return False #dark 100
    else:
        return -1 #out of bounds

def resample_particles(particles, weights, num_part):
    """
    Resamples particles with replacement based on their weights.

    Args:
        particles (list): List of particles
        weights (list): List of corresponding weights for each particle.
    
    Returns:
        list: A new set of resampled particles.
    """

    #check if self.particle_weights is populated
    if not weights:
        return particles #nothing to do but wait for more info
        #raise ValueError("Particle weights are not populated.")
    
    #convert the particle weights dictionary to a list of float values for np.random.choice
    weights_flt = list(weights.values())

    total_weight = sum(weights_flt)
    if total_weight > 0:
        weights_flt = [w / total_weight for w in weights_flt]  #Normalize weights so they sum to 1
    else:
        #If weights are all zero or there's some issue, use uniform weights
        weights_flt = [1.0 / num_part] * num_part
    
    # Resample particles based on their weights
    #np.random.choice(pick values from 0-299, 300 values, with replacement, with these weights)
    resampled_indices = np.random.choice(len(particles.markers), size=num_part, replace=True, p=weights_flt)
    
    resampled_particles = MarkerArray()
    
    #Use the resampled indices to fetch the actual particles and add them to the new MarkerArray
    count = 0
    for i in resampled_indices:
        resampled_particles.markers.append(copy.deepcopy(particles.markers[i]))
        resampled_particles.markers[count].id = count
        count += 1
    
    return resampled_particles

class ParticleFilterLocalization(Node):
    def __init__(self):
        super().__init__('particle_filter_localization')

        # Map and particle initialization
        self.map_width, self.map_height, self.resolution = 0, 0, 0
        self.num_particles = 3000
        self.particle_spread = 0.23
        self.particles = MarkerArray()
        self.particle_weights = {}
        self.map_data = OccupancyGrid()
        self.last_sensor_value = 0
        self.init_particles_bool = False
        self.last_movement_time = None
        # Subscribers
        self.map_subscriber = self.create_subscription(OccupancyGrid, '/floor', self.map_callback, 10)
        self.floor_sensor_subscriber = self.create_subscription(UInt8, '/floor_sensor', self.floor_sensor_callback, 10)
        # Synchronize robot twist and compass data
        self.cmd_vel_subscriber = self.create_subscription(Twist, '/cmd_vel', self.motion_update_callback, 10)
        self.compass_subscriber = self.create_subscription(Float32, '/compass', self.compass_callback, 10)

        # Publish particles
        self.particle_publisher = self.create_publisher(MarkerArray, '/particles', 10)
        self.timer = self.create_timer(0.2, self.publish_particles)

        #most likely robot marker/topic
        self.robot_index = 0 #index of the particle we think is the robot
        self.robot_marker = Marker()
        self.robot_marker_publisher = self.create_publisher(Marker, '/robot_marker', 10)
        self.est_pose_publisher = self.create_publisher(Pose2D, '/estimated_pose', 10)
        self.est_pose_timer = self.create_timer(0.5, self.update_robot_guess)
        self.init_robot_marker()

        #DEBUG
        self.test_pub = self.create_publisher(MarkerArray, '/test_marker', 10)
        self.debug_point = [0,0,0]

    def map_callback(self, msg):
        # Initialize map information and particle array
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.resolution = msg.info.resolution

        if len(self.particles.markers) == 0:
            self.map_data = msg
            self.particles = self.init_particles()

            # NOTE DEBUG for color testing
            # self.fill_points_in_grid()

    
    def floor_sensor_callback(self, floor_sensor_msg):
        #resampling callback
        
        #convert UInt8 to int before passing
        sensor_value = floor_sensor_msg.data

        #store last_sensor_value so that the
        #   forward projection callback (motion_update) has a value for weight assignment
        self.last_sensor_value = sensor_value

        #particle set replaced with resampled set
        if(self.init_particles_bool): # wait until initialization
            self.particles = resample_particles(self.particles, self.particle_weights, self.num_particles)         

    def motion_update_callback(self, cmd_vel_msg):
        #debug
        self.debug_point[0] = self.debug_point[0] + cmd_vel_msg.linear.x
        self.debug_point[1] = self.debug_point[1] + cmd_vel_msg.linear.y
        self.debug_point[2] = self.debug_point[2] + cmd_vel_msg.linear.z
        # self.get_logger().info(f'POINT: {self.debug_point}')

        curr_time = self.get_clock().now()
        if self.last_movement_time:
            duration = curr_time - self.last_movement_time
            duration = duration.nanoseconds / 10**9
        else:
            duration = 0.0

        self.last_movement_time = curr_time

        for particle in self.particles.markers:
            # Extract yaw from the particle's quaternion
            yaw = euler_from_quaternion(particle.pose.orientation)

            # Add noise and compute local velocities
            dx_local = duration * (cmd_vel_msg.linear.x + uniform(-self.particle_spread, self.particle_spread))
            dy_local = duration * (cmd_vel_msg.linear.y + uniform(-self.particle_spread, self.particle_spread))
            dtheta = duration * (cmd_vel_msg.angular.z + uniform(-0.01, 0.01))

            # Transform to global frame
            dx_global = dx_local * math.cos(yaw) - dy_local * math.sin(yaw)
            dy_global = dx_local * math.sin(yaw) + dy_local * math.cos(yaw)

            # Update position in the global frame
            particle.pose.position.x += dx_global
            particle.pose.position.y += dy_global

            # Add noise to orientation
            new_yaw = yaw + dtheta
            particle.pose.orientation = quaternion_from_euler(new_yaw)

            #assign weights from last sensor reading
            sensor_color, probability = classify_light_dark(self.last_sensor_value)
            particle_color = is_particle_on_light_spot(particle, self.map_data.data, self.resolution, self.map_width, self.map_height)
            
            if particle_color == -1:
                self.particle_weights[particle.id] = 0.0
                particle.color.r = 1.0
                particle.color.g = 0.0
                particle.color.b = 0.0
            elif particle_color:
                particle.color.r = 1.0
                particle.color.g = 1.0
                particle.color.b = 0.0
            else:
                particle.color.r = 0.0
                particle.color.g = 0.0
                particle.color.b = 1.0

            if(sensor_color == particle_color):
                #probability of being correct, always largest (i.e. 0.6)
                self.particle_weights[particle.id] = probability 
            else:
                #probability of being wrong, inverse (i.e. [1 - 0.6] = 0.4)
                self.particle_weights[particle.id] = 1 - probability

            # if particle is outside map, reduce exponentially            
            left, bottom, right, top = 0, 0, self.map_width * self.resolution, self.map_height * self.resolution
            particle_x, particle_y = particle.pose.position.x, particle.pose.position.y
            TUNNEL_CONST = 2
            diff = 0 # If particle is within bounds, diff will remain 0 and weight will not change (1 / e^(0 / 2) = 1 / e^0 = 1)

            if particle_x < left:
                diff = left - particle_x
            elif particle_x > right:
                diff = particle_x - right

            self.particle_weights[particle.id] *= 1 / math.exp(diff / TUNNEL_CONST)

            if particle_y < bottom:
                diff = bottom - particle_y
            elif particle_y > top:
                diff = particle_y - top                

            self.particle_weights[particle.id] *= 1 / math.exp(diff / TUNNEL_CONST)

    #NOTE: there's a chance we don't do this every compass update, but I think this is right
    def compass_callback(self, msg):
        for particle in self.particles.markers:
            particle.pose.orientation = quaternion_from_euler(msg.data)
        
    #NOTE: if this isn't working well, we can replace it with weight avg. of particles
    #calculates most likely robot position and publishes it to map & topic
    def update_robot_guess(self):
        best_particle = None
        best_pose = Pose2D()
        max_weight = 0.0
        for i, particle in enumerate(self.particles.markers):
            curr_weight = self.particle_weights[i]
            if curr_weight > max_weight:
                max_weight = curr_weight
                best_particle = particle
                self.robot_index = i

        if not best_particle:
            return

        ppose = best_particle.pose
        best_pose.x = ppose.position.x
        best_pose.y = ppose.position.y
        eulers = euler_from_quaternion(ppose.orientation)
        best_pose.theta = eulers
        self.robot_marker.pose = ppose
        self.robot_marker_publisher.publish(self.robot_marker)
        self.est_pose_publisher.publish(best_pose)

    def init_robot_marker(self):
        mark = self.robot_marker
        mark.header.frame_id = "map"
        mark.header.stamp = self.get_clock().now().to_msg()
        mark.id = 99999
        mark.type = Marker.SPHERE
        mark.action = Marker.ADD
        mark.scale.x = .3
        mark.scale.y = .3
        mark.scale.z = .3
        mark.color.r = 0.0
        mark.color.g = 1.0
        mark.color.b = 0.0
        mark.color.a = 1.0

    def init_particles(self):
        particle_list = MarkerArray()
        
        for i in range(self.num_particles):
            # Particle setup
            particle = Marker()
            particle.header.frame_id = "map"
            particle.header.stamp = self.get_clock().now().to_msg()
            particle.id = i
            particle.type = Marker.SPHERE
            particle.action = Marker.ADD
            self.particle_weights[particle.id] = 0 # each starts with initial weight of 0

            # Random position
            particle.pose.position.x = uniform(0, self.map_width) * self.resolution
            particle.pose.position.y = uniform(0, self.map_height) * self.resolution
            particle.pose.position.z = 0.0

            # Random orientation
            angle = uniform(0, 2 * math.pi)
            quaternion = quaternion_from_euler(angle)
            particle.pose.orientation = quaternion
            
            # Particle size and color
            particle.scale.x = 0.1
            particle.scale.y = 0.1
            particle.scale.z = 0.1
            particle.color.a = 1.0
            particle.color.r = 1.0
            particle.color.g = 0.0
            particle.color.b = 0.0

            particle_list.markers.append(particle)

        self.get_logger().info(f"Initialized {len(particle_list.markers)} particles...")
        self.init_particles_bool = True
        return particle_list
    
    def publish_particles(self):
        self.particle_publisher.publish(self.particles)

    #NOTE: this is for debugging purposes
    def fill_points_in_grid(self):
        # Create a marker array to visualize the grid
        marker_array = MarkerArray()
        border = 1
        counter = 0
        for r in range(-border, self.map_height + border):
            for c in range(-border, self.map_width + border):
                # create a marker in the middle of each cell
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.id = counter
                counter += 1
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = c * self.resolution + self.resolution / 2
                marker.pose.position.y = r * self.resolution + self.resolution / 2
                marker.pose.position.z = 0.0
                marker.pose.orientation = quaternion_from_euler(0, 0, 0)
                marker.scale.x = 0.4
                marker.scale.y = 0.4
                marker.scale.z = 0.4
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0

                marker_array.markers.append(marker)
        
        # Test color of each marker 
        for marker in marker_array.markers:
            color = is_particle_on_light_spot(marker, self.map_data.data, self.resolution, self.map_width, self.map_height)
            if color == -1:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            elif color:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0

        self.test_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = ParticleFilterLocalization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
