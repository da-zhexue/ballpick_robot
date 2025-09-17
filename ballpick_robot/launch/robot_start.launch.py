import launch
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    lidar_node = Node(
        package='m1ct_d2',
        executable='m1ct_d2',
        output='log',
    )

    robot_control_node = Node(
        package='ballpick_robot',
        executable='robot_control',
        output='screen',
    )

    imu_node = Node(
        package='wit_ros2_imu',
        executable='wit_ros2_imu',
        name='imu',
        remappings=[('/wit/imu', '/imu/data')],
        parameters=[{'port': '/dev/ttyUSB1'},
                    {"baud": 9600}],
        output="log"
    )

    cartographer_pkg_share = FindPackageShare(package='cartographer_ros').find('cartographer_ros')
    cartographer_launch_file_path = os.path.join(cartographer_pkg_share, 'launch', 'demo_backpack_2d_localization.launch.py')
    cartographer_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(cartographer_launch_file_path),
    )

    delayed_robot_control = TimerAction(
        period=5.0,  # 延迟5秒
        actions=[robot_control_node]
    )

    goal_pub = Node(
        package='ballpick_robot',
        executable='goal_pub_temp',
        output='screen',
    )

    return launch.LaunchDescription([
        lidar_node,
        imu_node,
        cartographer_node,
        delayed_robot_control,
        goal_pub
    ])