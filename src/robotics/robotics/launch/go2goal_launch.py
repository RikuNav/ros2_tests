import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

models_path = get_package_share_directory('minichallenges')
config = os.path.join(models_path, 'config', 'params.yaml')

def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
                            Node(
                                package='minichallenges',
                                executable='go2goal',
                                output='screen',
                                parameters=[ 
                                    config,
                                    {'use_sim_time': True}, 
                                ] 
                            ),
                            Node(
                                package='minichallenges',
                                executable='odom2wheelspeeds',
                                output='screen',
                                parameters=[ 
                                    config,
                                    {'use_sim_time': True}, 
                                ] 
                            ),
                            ExecuteProcess(cmd=['gz', 'sim', os.path.join(models_path, 'worlds', 'puzzlebot.sdf'), '--render-engine', 'ogre']),
                            ExecuteProcess(cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '/model/vehicle_blue/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry']),
                            ExecuteProcess(cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']),
                            ExecuteProcess(cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist']),
    ])