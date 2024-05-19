import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

package_name = 'minichallenges'
package_share = get_package_share_directory(package_name)
config = os.path.join(package_share, 'config', 'params.yaml')

def generate_launch_description() -> LaunchDescription:


    urdf_file_name = 'urdf/Puzzlebot.urdf'
    urdf = os.path.join(package_share, urdf_file_name)
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

    tf_resender = Node(
        package='minichallenges',
        executable='gz_tf_to_rviz_tf',
        name='gz_tf_to_rviz_tf',
        output='screen',
        parameters=[config, {'use_sim_time': True}],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'Puzzlebot', 
                   '-file', urdf,
                   '-x', '0', 
                   '-y', '0', 
                   '-z', '0.05'],
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock', 
                   '/model/Puzzlebot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                   '/model/Puzzlebot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
    )

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', 'empty.sdf'],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gz_bridge,
        tf_resender,
        rviz2,
        gazebo,
        spawn,
    ])