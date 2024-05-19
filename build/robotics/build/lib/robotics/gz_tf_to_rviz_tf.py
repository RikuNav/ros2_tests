from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped 
from tf2_ros import TransformBroadcaster 
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
import numpy as np

def quaternion_from_euler(roll, pitch, yaw): 
	cr = np.cos(roll/2)
	sr = np.sin(roll/2)
	cp = np.cos(pitch/2)
	sp = np.sin(pitch/2)
	cy = np.cos(yaw/2)
	sy = np.sin(yaw/2)

	q = np.zeros(4)
	q[0] = sr*cp*cy - cr*sp*sy
	q[1] = cr*sp*cy + sr*cp*sy
	q[2] = cr*cp*sy - sr*sp*cy
	q[3] = cr*cp*cy + sr*sp*sy
	
	return q 

class PublishWheelSpeeds(Node):
	def __init__(self):
		super().__init__('gz_tf_to_rviz_tf')

		#### Parameters ####
		self.declare_parameters(
			namespace='',
			parameters=[
				('wheel_length', rclpy.Parameter.Type.DOUBLE),
				('wheel_radius', rclpy.Parameter.Type.DOUBLE),
			]
		)
		self.chassis_length = self.get_parameter('wheel_length').get_parameter_value().double_value
		self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value

		#### TF Broadcaster ####
		self.tf_broadcaster = TransformBroadcaster(self)

		#### Left wheel tf Message ####
		self.left_wheel_tf = TransformStamped()
		#### Wheel Position ####
		self.left_wheel_tf.transform.translation.x = 0.041134
		self.left_wheel_tf.transform.translation.y = 0.074604
		self.left_wheel_tf.transform.translation.z = 0.02
		#### Left Wheel TFMessage Frames ####
		self.left_wheel_tf.header.frame_id = "base_link"
		self.left_wheel_tf.child_frame_id = "Left_Wheel"

		#### Left wheel tf Message ####
		self.right_wheel_tf = TransformStamped()
		#### Wheel Position ####
		self.right_wheel_tf.transform.translation.x = 0.041134
		self.right_wheel_tf.transform.translation.y = -0.074596
		self.right_wheel_tf.transform.translation.z = 0.02
		#### Left Wheel TFMessage Frames ####
		self.right_wheel_tf.header.frame_id = "base_link"
		self.right_wheel_tf.child_frame_id = "Right_Wheel"

		#### Subscribers ####
		self.create_subscription(TFMessage, '/model/Puzzlebot/tf', self.resend_tf, 10)
		self.create_subscription(Odometry, '/model/Puzzlebot/odometry', self.wheel_transforms, 10)

		#### Publisher ####
		self.tf = self.create_publisher(TFMessage, '/tf', 10)

		#### Variables ####
		#### Wheel Orientations ####
		self.left_wheel_yaw = 0.0
		self.right_wheel_yaw = 0.0

		### Previous Time ####
		self.previous_time = self.get_clock().now().nanoseconds/1000000000

	def resend_tf(self, msg):
		# Change transform frame_id to /world
		for i in range(len(msg.transforms)):
			msg.transforms[i].header.frame_id = 'world'
		# Change child frame_id to /base_link
		for i in range(len(msg.transforms)):
			msg.transforms[i].child_frame_id = 'base_link'
		self.tf.publish(msg)

	def wheel_transforms(self, msg):
		# Gets linear and angular velocities
		v = msg.twist.twist.linear.x
		w = msg.twist.twist.angular.z
		# Calculates wheel speeds
		wr = (2*v + w * self.chassis_length)/(2*self.wheel_radius)
		wl = (2*v - w * self.chassis_length)/(2*self.wheel_radius)
		# Sets the wheels time stamps
		self.left_wheel_tf.header.stamp = self.get_clock().now().to_msg()
		self.right_wheel_tf.header.stamp = self.get_clock().now().to_msg()
		# Stimates the wheels yaw
		self.left_wheel_yaw += wl*((self.get_clock().now().nanoseconds-self.previous_time)/1000000000)
		self.right_wheel_yaw += wr*((self.get_clock().now().nanoseconds-self.previous_time)/1000000000)
		self.previous_time = self.get_clock().now().nanoseconds
		# Gets the quaternion from euler angles
		left_quaternion = quaternion_from_euler(1.5708, self.left_wheel_yaw, 0) 
		right_quaternion = quaternion_from_euler(1.5708, self.right_wheel_yaw, 0) 
		self.left_wheel_tf.transform.rotation.y = left_quaternion[1] 
		self.left_wheel_tf.transform.rotation.z = left_quaternion[2] 
		self.left_wheel_tf.transform.rotation.w = left_quaternion[3] 
		self.left_wheel_tf.transform.rotation.x = left_quaternion[0] 
		self.right_wheel_tf.transform.rotation.x = right_quaternion[0] 
		self.right_wheel_tf.transform.rotation.y = right_quaternion[1] 
		self.right_wheel_tf.transform.rotation.z = right_quaternion[2] 
		self.right_wheel_tf.transform.rotation.w = right_quaternion[3] 
		# Publish the transforms
		self.tf_broadcaster.sendTransform(self.left_wheel_tf) 
		self.tf_broadcaster.sendTransform(self.right_wheel_tf) 

def main(args=None): 
	rclpy.init(args=args) 
	f_p = PublishWheelSpeeds() 
	rclpy.spin(f_p) 
	f_p.destroy_node() 
	rclpy.shutdown() 

if __name__ == '__main__': 
	main() 