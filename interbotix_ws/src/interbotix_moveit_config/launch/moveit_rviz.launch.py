import os
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package paths
    interbotix_moveit_config = get_package_share_directory('interbotix_moveit_config')
    rviz_config_file = os.path.join(interbotix_moveit_config, 'rviz', 'moveit.rviz')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument("robot_description", default_value="robot_description"),
        DeclareLaunchArgument("robot_description_semantic", default_value="robot_description_semantic"),

        # Launch MoveIt! Move Group node
        launch_ros.actions.Node(
            package="moveit_ros_move_group",
            executable="move_group",
            name="move_group",
            output="screen",
            parameters=[{
                "robot_description": LaunchConfiguration("robot_description"),
                "robot_description_semantic": LaunchConfiguration("robot_description_semantic"),
                "kinematics_solver": "trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin",
                "kinematics_solver_timeout": 0.005
            }]
        ),

        # Start RViz for visualization
        launch_ros.actions.Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_file]
        ),
    ])

