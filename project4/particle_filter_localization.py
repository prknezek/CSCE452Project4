import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid
from random import uniform
from geometry_msgs.msg import Quaternion, Pose2D, Twist
import math
from example_interfaces.msg import Float32

from example_interfaces.msg import UInt8

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
        return True, 1.0
    elif dark_range[0] <= value <= dark_range[1]:
        return False, 1.0
    else:
        return "unknown", 0.0  #NaN and whatnot




class ParticleFilterLocalization(Node):
    def __init__(self):
        super().__init__('particle_filter_localization')

        # Map and particle initialization
        self.map_width, self.map_height, self.resolution = 0, 0, 0
        self.num_particles = 300
        self.particles = MarkerArray()
        self.particle_weights = {}
        self.map_data = OccupancyGrid()
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
        self.est_pose_timer = self.create_timer(2.0, self.update_robot_guess)
        self.init_robot_marker()

        # test
        sensor_value = 123
        color, probability = classify_light_dark(sensor_value)
        color_text = ""
        if(color):
            color_text = "Light"
        else:
            color_text = "Dark"
        self.get_logger().info(f"Value: {sensor_value}, Color: {color_text}, Probability: {probability:.2f}")

    def map_callback(self, msg):
        # Initialize map information and particle array
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.resolution = msg.info.resolution

        if len(self.particles.markers) == 0:
            self.map_data = msg #send map_data?
            self.particles = self.init_particles()

    
    def floor_sensor_callback(self, floor_sensor_msg):
        #resampling callback
        
        #convert UInt8 to int before passing
        sensor_value = floor_sensor_msg.data
        
        #check color of sensor
        color, probability = classify_light_dark(sensor_value)
        self.get_logger().info("floor_sensor_callback ----")
        #check color of particles
        if(self.map_data.data):
            self.get_logger().info(f"map_data.data: {self.map_data.data}") #goes left to right from bottom to top
            #if colors match
                #increase weight by probability
            #else
                #decrease weight by probability

            #if weight is < 1
                #move particle to another particle's position with greater weight

        



    def motion_update_callback(self, cmd_vel_msg):
        for particle in self.particles.markers:
            # Extract yaw from the particle's quaternion
            yaw = euler_from_quaternion(particle.pose.orientation)

            # Add noise and compute local velocities
            dx_local = cmd_vel_msg.linear.x + uniform(-0.1, 0.1)
            dy_local = cmd_vel_msg.linear.y + uniform(-0.1, 0.1)
            dtheta = cmd_vel_msg.angular.z + uniform(-0.01, 0.01)

            # Transform to global frame
            dx_global = dx_local * math.cos(yaw) - dy_local * math.sin(yaw)
            dy_global = dx_local * math.sin(yaw) + dy_local * math.cos(yaw)

            # Update position in the global frame
            particle.pose.position.x += dx_global
            particle.pose.position.y += dy_global

            # Add noise to orientation
            new_yaw = yaw + dtheta
            particle.pose.orientation = quaternion_from_euler(new_yaw)

    #NOTE: there's a chance we don't do this every compass update, but I think this is right
    def compass_callback(self, msg):
        self.get_logger().info(f'COMP ASS MESSAGE: {msg}')
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
            self.particle_weights[particle.id] = 1.0 # each starts with initial weight of 1

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
        return particle_list
    
    def publish_particles(self):
        self.particle_publisher.publish(self.particles)

def main(args=None):
    rclpy.init(args=args)
    node = ParticleFilterLocalization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
