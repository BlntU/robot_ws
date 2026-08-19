from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument 
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition

def generate_launch_description():

    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.033"
    )
    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.17"
    )
    
    # True: Simple Controller 
    # False : diff_drive_controller
    use_custom_controller_arg = DeclareLaunchArgument( 
        "use_custom_controller",
        default_value="True"
    )

    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    use_custom_controller = LaunchConfiguration("use_custom_controller")

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    diff_drive_control = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "robot_diff_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        condition=UnlessCondition(use_custom_controller),
    )
    

    simple_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "simple_velocity_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        condition=IfCondition(use_custom_controller),
    )

    simple_controller_cpp = Node(
        package="robot_controller",
        executable="simple_controller",
        parameters=[{"wheel_radius":wheel_radius,
                    "wheel_separation":wheel_separation}],
        condition=IfCondition(use_custom_controller),
    )

    teleop = Node(
        package="robot_controller",
        executable="vel_bridge"
    )

    return LaunchDescription([
        use_custom_controller_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        diff_drive_control,
        simple_controller_cpp,
        joint_state_spawner,
        simple_controller,
        teleop
    ])