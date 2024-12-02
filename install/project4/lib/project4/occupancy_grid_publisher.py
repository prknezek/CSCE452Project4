import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from rclpy.timer import Timer

class OccupancyGridPublisher(Node):
    def __init__(self):
        super().__init__('occupancy_grid_publisher')
        
        self.declare_parameter('map_file', '')
        map_file = self.get_parameter('map_file').get_parameter_value().string_value
        
        if not map_file:
            self.get_logger().error('No map file provided! Use --ros-args -p map_file:=<path_to_world_file>')
            return

        self.map_data, self.resolution = self.parse_world_file(map_file) #Load the map

        self.publisher = self.create_publisher(OccupancyGrid, '/floor', 10)
        self.timer = self.create_timer(5.0, self.publish_map)

        
    def parse_world_file(self, map_file):
        #Parses the .world file and returns the map grid and resolution. 
        self.get_logger().info(f"Loading map file: {map_file}")
        resolution = 1.2  # default resolution from cave.world
        map_data = []
        map_section = False

        try:
            with open(map_file, 'r') as file:
                lines = file.readlines()

            for line in lines:
                line = line.strip()
                if line.startswith('resolution:'):
                    resolution = float(line.split(':')[1].strip())
                    self.get_logger().info(f"Map resolution: {resolution}")
                elif line.startswith('map:'):
                    map_section = True  #start processing the map section
                    self.get_logger().info("Found map section, starting to parse...")
                    continue
                elif map_section and line:  #process only lines in the map section
                    map_row = [100 if char == '#' else 0 if char == '.' else -1 for char in line]
                    map_data.append(map_row)

            if not map_data:
                self.get_logger().error("Parsed map data is empty! Check the map section in the file.")

            map_data.reverse()  #Reverse to match with OccupancyGrid standards
            return map_data, resolution

        except FileNotFoundError:
            self.get_logger().error(f"Map file not found: {map_file}")
            return [], 1.2


    def publish_map(self):
        flattened_map = [cell for row in self.map_data for cell in row]
        width = len(self.map_data[0])
        height = len(self.map_data)
        
        #create OccupancyGrid message
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header = Header()
        occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid.header.frame_id = 'map'

        occupancy_grid.info.resolution = self.resolution
        occupancy_grid.info.width = width
        occupancy_grid.info.height = height
        occupancy_grid.info.origin = Pose()  # origin at (0, 0, 0)

        occupancy_grid.data = flattened_map
        
        # publish the map
        self.publisher.publish(occupancy_grid)
        #self.get_logger().info('OccupancyGrid published')

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
