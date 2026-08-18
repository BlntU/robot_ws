from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument 
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.033"
    )
    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.17"
    )

    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    simple_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "simple_velocity_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    simple_controller_cpp = Node(
        package="robot_controller",
        executable="simple_controller",
        parameters=[{"wheel_radius":wheel_radius,
                    "wheel_separation":wheel_separation}]
    )

    teleop = Node(
        package="robot_controller",
        executable="vel_bridge"
    )

    return LaunchDescription([
        wheel_radius_arg,
        wheel_separation_arg,
        simple_controller_cpp,
        joint_state_spawner,
        simple_controller,
        teleop
    ])