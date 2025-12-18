from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    tb3_nav_dir = get_package_share_directory('turtlebot3_navigation')
    gazebo_launch = os.path.join(tb3_nav_dir, 'launch', 'turtlebot3_dqn_stage2.launch.py')

    set_env_variable = SetEnvironmentVariable(
                            name='TURTLEBOT3_MODEL',
                            value='burger'
                        )

    include_turtlebot3_launch = IncludeLaunchDescription(
                                    PythonLaunchDescriptionSource(gazebo_launch),
                                )
    
    wall_follower_node = Node(
            package='turtlebot3_navigation',
            executable='wall_follower',
            name='wall_follower',
        )

    return LaunchDescription([
        set_env_variable,
        include_turtlebot3_launch,
        wall_follower_node
    ])
