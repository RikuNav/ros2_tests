import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D 
from std_msgs.msg import Float32

class Go2Goal(Node):
    def __init__(self) -> None:
        super().__init__('go2goal')
        #### PARAMETERS ####
        self.declare_parameters(
            namespace='',
            parameters=[
                ('KvP', rclpy.Parameter.Type.DOUBLE),
                ('KthetaP', rclpy.Parameter.Type.DOUBLE),
                ('wheel_length', rclpy.Parameter.Type.DOUBLE),
                ('wheel_radius', rclpy.Parameter.Type.DOUBLE),
                ('tolerance', rclpy.Parameter.Type.DOUBLE),
                ('linear_acc', rclpy.Parameter.Type.DOUBLE),
                ('angular_acc', rclpy.Parameter.Type.DOUBLE),
            ]
        )
        #### ROBOT PARAMETERS ####
        self.r = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.L = self.get_parameter('wheel_length').get_parameter_value().double_value
        self.KvP = self.get_parameter('KvP').get_parameter_value().double_value
        self.KthetaP = self.get_parameter('KthetaP').get_parameter_value().double_value
        self.tolerance = self.get_parameter('tolerance').get_parameter_value().double_value
        self.linear_acc = self.get_parameter('linear_acc').get_parameter_value().double_value
        self.angular_acc = self.get_parameter('angular_acc').get_parameter_value().double_value
        #### ROBOT VARIABLES ####
        self.wheels_velocities = [0, 0]
        self.pose = [0, 0, 0]
        self.goal_pose = [0, 0, 0]
        self.cmd_vel = Twist()
        self.first_movement = True
        #### PUBLISHERS ####
        self.robot_pose = self.create_publisher(Pose2D, 'pose', 10)
        self.robot_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        #### SUBSCRIBERS ####
        self.wr_subscriber = self.create_subscription(Float32, 'VelocityEncR', self.callback_wr, 10)
        self.wl_subscriber = self.create_subscription(Float32, 'VelocityEncL', self.callback_wl, 10)
        self.goal = self.create_subscription(Pose2D, 'goal', self.goal_callback, 10)
        #### TIMER ####
        self.create_timer(0.1, self.update_odometry)
        self.get_logger().info('Go2Goal node has been created.')

    def callback_wl(self, msg):
        self.wheels_velocities[0] = msg.data

    def callback_wr(self, msg):
        self.wheels_velocities[1] = msg.data
        
    def update_odometry(self):
        if self.first_movement:
            self.previous_time = self.get_clock().now().nanoseconds
            self.first_movement = False
        else:
            v = (self.wheels_velocities[1] + self.wheels_velocities[0]) * self.r / 2.0
            w = (self.wheels_velocities[1] - self.wheels_velocities[0]) * self.r / self.L
            self.current_time = self.get_clock().now().nanoseconds  
            dt = float(self.current_time  - self.previous_time)/(10.0**9)
            self.previous_time = self.current_time
            self.pose[0] += v * np.cos(self.pose[2]) * dt
            self.pose[1] += v * np.sin(self.pose[2]) * dt
            self.pose[2] += w * dt

            ed = np.sqrt((self.goal_pose[0] - self.pose[0])**2 + (self.goal_pose[1] - self.pose[1])**2)
            theta = np.arctan2(self.goal_pose[1] - self.pose[1], self.goal_pose[0] - self.pose[0])
            e_theta = theta - self.pose[2]
            e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))

            lineal_vel = self.KvP * ed
            angular_vel = self.KthetaP * e_theta
            if lineal_vel-v > self.linear_acc:
                lineal_vel = v+self.linear_acc*dt

            if angular_vel-w > self.angular_acc:
                angular_vel = w+self.angular_acc*dt

            if abs(ed) > self.tolerance:
                if abs(e_theta) > np.pi/16:
                    self.cmd_vel.linear.x = 0.0
                    self.cmd_vel.angular.z = angular_vel
                else:
                    self.cmd_vel.linear.x = lineal_vel
                    self.cmd_vel.angular.z = angular_vel
            else:
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0

            self.robot_vel.publish(self.cmd_vel)
            self.pose_array_2_pose2d(self.pose)

    def goal_callback(self, msg):
        self.goal_pose = [msg.x, msg.y, msg.theta]
        self.first_movement = True

    def pose_array_2_pose2d(self, pose):
        msg = Pose2D()
        msg.x = pose[0]
        msg.y = pose[1]
        msg.theta = pose[2]
        self.robot_pose.publish(msg)

def main():
    rclpy.init()
    go2goal = Go2Goal()
    rclpy.spin(go2goal)
    go2goal.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()