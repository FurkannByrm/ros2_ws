import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Paket paylaşım dizininden URDF dosyasını al
    pkg_share = get_package_share_directory('gazebo_control')
    urdf_file = os.path.join(pkg_share, 'urdf', 'amr.xacro')

    # Xacro dosyasını çözümle (URDF'ye dönüştür)
    xacro = launch.substitutions.LaunchConfiguration('xacro_file')
    urdf = launch.substitutions.LaunchConfiguration('robot_description')

    # Gazebo parametrelerini oluştur
    robot_description = open(urdf_file, 'r').read()

    # Launch dosyasını oluştur
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_description', default_value=robot_description,
            description='Robot URDF description'),

        # Robotun URDF'sini robot_description parametresine yükle
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
            remappings=[('/robot_description', 'robot_description')]
        ),

        # Gazebo'nun başlatılması
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=['-topic', '/robot_description', '-entity', 'amr_robot'],
        ),

        # LIDAR sensörünü simüle et
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_lidar_left',
            output='screen',
            arguments=['-topic', '/robot_description', '-entity', 'lidar_left', '-z', '0.2', '-x', '0.5', '-y', '0.25'],
        ),
        
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_lidar_right',
            output='screen',
            arguments=['-topic', '/robot_description', '-entity', 'lidar_right', '-z', '0.2', '-x', '0.5', '-y', '-0.25'],
        ),
        
        # Gazebo GUI
        Node(
            package='gazebo_ros',
            executable='gzserver',
            name='gzserver',
            output='screen',
            arguments=['--verbose']
        ),

        Node(
            package='gazebo_ros',
            executable='gzclient',
            name='gzclient',
            output='screen'
        ),

        # RViz'i başlat
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'robot.rviz')],
        ),
    ])
