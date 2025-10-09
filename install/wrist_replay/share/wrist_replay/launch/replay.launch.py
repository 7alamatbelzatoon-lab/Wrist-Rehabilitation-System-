from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    pkg_share = Path(get_package_share_directory('wrist_replay'))
    urdf_path = pkg_share / 'urdf' / 'wrist.urdf'
    # Remove any BOM or leading whitespace/newlines that can break the XML declaration rule
    urdf_text = urdf_path.read_text(encoding='utf-8').lstrip('\ufeff \t\r\n')

    csv_arg = DeclareLaunchArgument(
        'csv_path',
        default_value=str(Path('/home/anas/ros2_ws/adaptive_log.csv'))
    )
    loop_arg = DeclareLaunchArgument('loop', default_value='false')

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf_text}]
    )

    replay = Node(
        package='wrist_replay',
        executable='replay_node',
        name='replay_node',
        output='screen',
        parameters=[
            {'csv_path': LaunchConfiguration('csv_path')},
            {'loop': LaunchConfiguration('loop')},
        ]
    )

    rviz = Node(package='rviz2', executable='rviz2', name='rviz2')

    return LaunchDescription([csv_arg, loop_arg, robot_state_pub, replay, rviz])
