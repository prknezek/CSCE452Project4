from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Declare a launch argument for the map_file parameter
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='project4_data/cave.world',
        description='Path to the map file'
    )

    bag = DeclareLaunchArgument("bag",
                                   default_value=TextSubstitution(text="DEFAULT"))

    playback_speed = DeclareLaunchArgument("rate",
                                           default_value=TextSubstitution(text="1.0"))

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

    play_bag = TimerAction(
            period = 1.0,
            actions = [ExecuteProcess(
                cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag'), '--rate', LaunchConfiguration('rate')]
                )
            ]
            )

    bagger = Node(package='project4',
                  namespace='Bagger',
                  executable="Bagger",
                  name="Bagger",
                  parameters=[{
                      "bag_out" : LaunchConfiguration('bag_out')
                      }]
                  )
    # Return the launch description
    return LaunchDescription([
        map_file_arg,
        bag,
        playback_speed,
        occupancy_grid_publisher_node,
        particle_filter_localization_node,
        play_bag,
        bagger
    ])
