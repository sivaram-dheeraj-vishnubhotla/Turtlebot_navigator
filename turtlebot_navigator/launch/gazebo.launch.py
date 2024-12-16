from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    package = Path(get_package_share_directory('turtlebot_navigator'))
    gazebo = Path(get_package_share_directory('ros_gz_sim'))
    world = PathJoinSubstitution([str(package / 'worlds'), LaunchConfiguration('world')])
    server_only = PythonExpression([
        "'-s --headless-rendering ' if '", LaunchConfiguration('gui'),
        "'.lower() == 'false' else ''",
    ])

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='arena.world'),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(package / 'models')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo / 'launch' / 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': ['-r ', server_only, '"', world, '"'],
                'on_exit_shutdown': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package / 'launch' / 'robot_frames.launch.py')),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': str(package / 'config' / 'ros_gazebo_topics.yaml')}],
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package / 'launch' / 'spawn_robot.launch.py')),
            launch_arguments={
                'x_pose': LaunchConfiguration('x_pose'),
                'y_pose': LaunchConfiguration('y_pose'),
            }.items(),
        ),
    ])
