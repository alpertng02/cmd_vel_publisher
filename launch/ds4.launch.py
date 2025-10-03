import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources  import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource



def generate_launch_description():
    
    package_name='cmd_vel_publisher' #<--- CHANGE ME

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory(package_name))

    
    ds4_cmd_vel = Node(
            package=package_name,
            executable='joystick_to_cmdvel',
            name='joystick_to_cmdvel',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, os.path.join(pkg_path, 'config', 'ds4_config.yaml')],
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        ds4_cmd_vel,
    ])