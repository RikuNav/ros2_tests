import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import UnlessCondition, IfCondition
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

models_path = get_package_share_directory('minichallenges')

def generate_launch_description() -> LaunchDescription:
    
    urdf_file_name = 'urdf/Puzzlebot.urdf'
    urdf = os.path.join(models_path, urdf_file_name)
    assert os.path.exists(urdf), "URDF file not found at %s" % urdf
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
    
    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        name = 'robot_state_publisher',
        output = 'screen',
        parameters = [{'robot_description': robot_desc}],
        arguments = [urdf],
    )

    joint_state_publisher_node = Node(
        package = 'joint_state_publisher',
        executable = 'joint_state_publisher',
        name = 'joint_state_publisher',
        condition = UnlessCondition(LaunchConfiguration('gui')),
    )

    joint_state_publisher_gui_node = Node(
        package = 'joint_state_publisher_gui',
        executable = 'joint_state_publisher_gui',
        name = 'joint_state_publisher_gui',
        condition = IfCondition(LaunchConfiguration('gui')),
    )

    rviz_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        name = 'rviz2',
        output = 'screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='gui', default_value='false', description='Flag to enable joint_state_publisher_gui'),
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])