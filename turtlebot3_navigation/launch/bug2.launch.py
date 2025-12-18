from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    gazebo_launch = os.path.join(tb3_gazebo_dir, 'launch', 'turtlebot3_dqn_stage4.launch.py')

    set_env_variable = SetEnvironmentVariable(
                            name='TURTLEBOT3_MODEL',
                            value='burger'
                        )

    include_turtlebot3_launch = IncludeLaunchDescription(
                                    PythonLaunchDescriptionSource(gazebo_launch),
                                    launch_arguments={'x_pose': '0.0', 'y_pose': '0.0'}.items()
                                )
    
    parameter_file_path = os.path.join(get_package_share_directory('turtlebot3_navigation'),'config','params.yaml')
    
    bug_node = Node(
            package='turtlebot3_navigation',
            executable='bug2',
            name='Bug2',
            parameters=[parameter_file_path],
        )

    return LaunchDescription([
        set_env_variable,
        include_turtlebot3_launch,
        bug_node,
    ])
