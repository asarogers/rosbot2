from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get the package share directory
    pkg_share = FindPackageShare(package='rosbot')

    # Get the XACRO file path
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'rosbot.urdf.xacro'])

    rviz_config = PathJoinSubstitution([pkg_share, "config", "rviz_config.rviz"])

    # Process the XACRO file and ensure it is treated as a string
    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # Create a dictionary with the robot description parameter
    robot_description = {'robot_description': robot_description_content}

    # Define and return the launch description
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_node",
            arguments=['-d', rviz_config],

        )
    ])
