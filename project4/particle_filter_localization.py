import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid
from random import uniform
from geometry_msgs.msg import Quaternion
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

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
        # Subscribers
        self.map_subscriber = self.create_subscription(OccupancyGrid, '/floor', self.map_callback, 10)
        # Synchronize robot twist and compass data
        self.cmd_vel_subscriber = self.create_subscription(Twist, '/cmd_vel', self.motion_update_callback, 10)
        # self.compass_subscriber = self.create_subscription(Float32, '/compass', self.compass_callback, 10)

        # Publish particles
        self.particle_publisher = self.create_publisher(MarkerArray, '/particles', 10)
        self.timer = self.create_timer(0.2, self.publish_particles)
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
            self.particles = self.init_particles()

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
