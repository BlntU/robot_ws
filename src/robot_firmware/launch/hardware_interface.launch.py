from launch import LaunchDescription 
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument ,SetEnvironmentVariable ,IncludeLaunchDescription
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value = os.path.join(get_package_share_directory("robot_description"), "urdf", "robot.urdf.xacro"),
        description= "Path for urdf file"

    )
    
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model"), " is_sim:=False"]), value_type=str)

    robot_state_publisher = Node (
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            os.path.join(get_package_share_directory("robot_controller"), "config", "robot_controllers.yaml")
        ]
    )             

    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        controller_manager
    ])