from launch import LaunchDescription 
from launch.actions import IncludeLaunchDescription
import os
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

    gazebo = IncludeLaunchDescription(
                os.path.join(
                    get_package_share_directory("robot_description"), 
                    "launch", 
                    "gazebo.launch.py")
    )

    controller = IncludeLaunchDescription(
                os.path.join(
                    get_package_share_directory("robot_controller"), 
                    "launch", 
                    "controller.launch.py"),
                    launch_arguments={
                        "use_custom_controller": "True"
                    }.items()
    )

    return LaunchDescription([
        gazebo,
        controller
    ])