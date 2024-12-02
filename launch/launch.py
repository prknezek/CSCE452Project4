from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare a launch argument for the map_file parameter
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='/home/prknezek/VSCode/CSCE752_Robotics/Project4/cave.world',
        description='Path to the map file'
    )

    # Define the occupancy_grid_publisher node
    occupancy_grid_publisher_node = Node(
        package='project4',
        executable='occupancy_grid_publisher',
        name='occupancy_grid_publisher',
        parameters=[{'map_file': LaunchConfiguration('map_file')}],
        output='screen'
    )

    # Define the particle_filter_localization node
    particle_filter_localization_node = Node(
        package='project4',
        executable='particle_filter_localization',
        name='particle_filter_localization',
        output='screen'
    )

    # Return the launch description
    return LaunchDescription([
        map_file_arg,
        occupancy_grid_publisher_node,
        particle_filter_localization_node,
    ])
