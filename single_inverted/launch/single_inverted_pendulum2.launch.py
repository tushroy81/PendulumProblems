from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():

    package_dir = os.path.dirname(os.path.abspath(__file__))
    rviz_config_file = os.path.join(package_dir, '..', 'config', 'config.rviz')
    
    visualizer = Node(
        package = 'rviz2',
        executable = 'rviz2',
        # name='rviz2',
        arguments = ['-d', rviz_config_file]

    )

    dynamics = Node(
        package = 'single_inverted',
        executable = 'dynamics_sim2'
    )
    
    controller = Node(
        package = 'single_inverted',
        executable = 'controller2_pend'
    )

    return LaunchDescription([
        visualizer,
        dynamics,
        controller
    ])
