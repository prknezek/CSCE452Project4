import rclpy
import rclpy.node
from rclpy.node import Node
import subprocess

class Bagger(Node):
    def __init__(self):
        super().__init__('Bagger')
        
        self.declare_parameter('bag_out', 'NONE')
        bag_file = self.get_parameter('bag_out').get_parameter_value().string_value
        
        #log the bag file name
        self.get_logger().info(f'bag_out file name: {bag_file}')
        
        #Start the recording process
        self.start_recording(bag_file)

    def start_recording(self, bag_file):
        try:
            #record all topics
            command = ["ros2", "bag", "record", "-o", bag_file, "-a"]
            self.recording_process = subprocess.Popen(command)
            self.get_logger().info("Recording started")
        
        except Exception as e:
            self.get_logger().error(f"Failed to start recording: {e}")

    def stop_recording(self):
        if hasattr(self, 'recording_process') and self.recording_process:
            self.recording_process.terminate()
            self.get_logger().info("Recording stopped.")

def main():
    rclpy.init()
    node = Bagger()
    rclpy.spin(node)
    
    #the recording stops when the node is shut down
    node.stop_recording()
    node.destroy_node()
    rclpy.shutdown()