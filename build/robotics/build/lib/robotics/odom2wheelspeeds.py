from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

class PublishWheelSpeeds(Node):
  def __init__(self):
    super().__init__('odom2wheelspeeds')
    #### PARAMETERS ####
    self.declare_parameters(
        namespace='',
        parameters=[
            ('wheel_length', rclpy.Parameter.Type.DOUBLE),
            ('wheel_radius', rclpy.Parameter.Type.DOUBLE),
        ]
    )
    self.create_subscription(Odometry, '/model/vehicle_blue/odometry', self.calculate_wheel_speeds, 10)

    self.wr_publisher = self.create_publisher(Float32, 'VelocityEncR', 10)
    self.wl_publisher = self.create_publisher(Float32, 'VelocityEncL', 10)

    self.L = self.get_parameter('wheel_length').get_parameter_value().double_value
    self.r = self.get_parameter('wheel_radius').get_parameter_value().double_value
    self.wr = Float32()
    self.wl = Float32()

    self.dt = 0.04
    self.timer = self.create_timer(self.dt, self.on_timer)

  def on_timer(self):
    self.wr_publisher.publish(self.wr)
    self.wl_publisher.publish(self.wl)

  def calculate_wheel_speeds(self, msg):
    v = msg.twist.twist.linear.x
    w = msg.twist.twist.angular.z
    self.wr.data = (2*v + w * self.L)/(2*self.r)
    self.wl.data = (2*v - w * self.L)/(2*self.r)

def main(args=None): 
  rclpy.init(args=args) 
  f_p = PublishWheelSpeeds() 
  rclpy.spin(f_p) 
  f_p.destroy_node() 
  rclpy.shutdown() 

if __name__ == '__main__': 
    main() 