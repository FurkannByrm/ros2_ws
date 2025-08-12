from launch import LaunchDescription
from launch_ros.actions import Node
import yaml
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def load_yaml(package_name, file_path):
    with open(
        os.path.join(get_package_share_directory(package_name), file_path), 'r'
    ) as f:
        return yaml.safe_load(f)

def generate_launch_description():
    # Xacro dosyasını işleyerek urdf stringi oluştur
    xacro_file = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config/panda.urdf.xacro"
    )
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {"robot_description": robot_description_config.toxml()}

    # SRDF dosyasını string olarak oku ve kontrol et
    srdf_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config/panda.srdf"
    )
    assert os.path.exists(srdf_path), f"SRDF file not found: {srdf_path}"
    with open(srdf_path, 'r') as f:
        srdf_string = f.read()
    assert len(srdf_string.strip()) > 0, f"SRDF file is empty: {srdf_path}"
    robot_description_semantic = {"robot_description_semantic": srdf_string}

    kinematics_yaml = load_yaml("moveit_resources_panda_moveit_config", "config/kinematics.yaml")
    joint_limits = load_yaml("moveit_resources_panda_moveit_config", "config/joint_limits.yaml")
    ompl_planning = load_yaml("moveit_resources_panda_moveit_config", "config/ompl_planning.yaml")

    planning_volume = {"planning_volume": {"min_x": -2.0, "min_y": -2.0, "min_z": -2.0, "max_x": 2.0, "max_y": 2.0, "max_z": 2.0}}

    return LaunchDescription([
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            output="both",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "world", "panda_link0"],
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics_yaml,
                joint_limits,
                ompl_planning,
                planning_volume
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="log",
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics_yaml,
                joint_limits,
                ompl_planning,
                planning_volume
            ],
        ),
        Node(
            package="moveit2_samples",
            executable="moveit_example",
            output="screen",
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics_yaml,
                joint_limits,
                ompl_planning,
                planning_volume
            ],
        ),
    ])