import rclpy
import numpy as np
import tf
from quaternions import quaternion_from_matrix
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped 
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster

class Joint():
    def __init__(self, **kwargs):
        self.__theta_offset = kwargs.get('theta_offset', 0)
        self.__d_offset = kwargs.get('d_offset', 0)
        self.__l_offset = kwargs.get('l_offset', 0)
        self.__alpha_offset = kwargs.get('alpha_offset', 0)

    def get_joint_type(self) -> str:
        return 'JOINT'
    
    def get_offsets(self) -> tuple:
        return (self.__theta_offset, self.__d_offset, self.__l_offset, self.__alpha_offset)

class RevoluteJoint(Joint):
    def get_joint_type(self) -> str:
        return 'REVOLUTE'

class PrismaticJoint(Joint):
    def get_joint_type(self) -> str:
        return 'PRISMATIC'

class Robot():
    def parse2float(self, joint_type: str) -> float:
        return self.parse_dict[joint_type]
    
    def parse2string(self, joint_type: float) -> str:
        return {v: k for k, v in self.parse_dict.items()}[joint_type]

    def __init__(self, *args) -> None:
        self.joint_transforms = []
        self.end_effector = np.eye(4)

        self.joints_number = len(args)

        self.parse_dict = {'REVOLUTE': 1, 'PRISMATIC': 2}
        self.offset_parameters = np.zeros((self.joints_number, 5))
        self.offset_vector = [joint.get_offsets() for joint in args]

        for idx, joint in enumerate(args):
            self.offset_parameters[idx] = np.array([self.parse2float(joint.get_joint_type()), 
                                                self.offset_vector[idx][0], 
                                                self.offset_vector[idx][1], 
                                                self.offset_vector[idx][2], 
                                                self.offset_vector[idx][3],
                                                ])
            if self.parse2string(self.offset_parameters[idx][0]) != 'REVOLUTE' and self.parse2string(self.offset_parameters[idx][0]) != 'PRISMATIC':
                raise ValueError('Joint type not supported, please use Revolute or Prismatic')
            
    def compute_fk(self, movement_vector: np.array) -> None:
        self.joint_transforms = []
        self.end_effector = np.eye(4)
        for idx, dh_parameter in enumerate(self.offset_parameters):
            dh_parameter_copy = dh_parameter.copy()
            if self.parse2string(dh_parameter_copy[0]) == 'REVOLUTE':
                dh_parameter_copy[1] = self.offset_parameters[idx][1]+movement_vector[idx]
            elif self.parse2string(dh_parameter_copy[0]) == 'PRISMATIC':
                dh_parameter_copy[3] = self.offset_parameters[idx][3]+movement_vector[idx]
            print(dh_parameter_copy)
            print(tf.trotz(dh_parameter_copy[1]))
            print(tf.transz(dh_parameter_copy[2])@tf.transx(dh_parameter_copy[3])@tf.trotx(dh_parameter_copy[4]))
            self.joint_transforms.append(tf.trotz(dh_parameter_copy[1])@tf.transz(dh_parameter_copy[2])@tf.transx(dh_parameter_copy[3])@tf.trotx(dh_parameter_copy[4]))
            self.end_effector = self.end_effector@self.joint_transforms[-1]
        print(self.joint_transforms)

    def get_joints_number(self) -> int:
        return self.joints_number
    
    def get_joint_transforms(self) -> np.array:
        return self.joint_transforms

class VariableRobotKinematics(Node):
    def __init__(self, robot):
        super().__init__('variable_robot_kinematics')
        self.robot = robot
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_messages = [TransformStamped() for joint in range(self.robot.get_joints_number())]

        self.movement_subscriber = self.create_subscription(Float64MultiArray, 'movement_vector', self.move_robot, 10) 

        self.get_logger().info('Variable Robot Kinematics Node has been started')

        self.move_robot(Float64MultiArray(data=[0 for joint in range(self.robot.get_joints_number())]))

    def move_robot(self, msg) -> None:
        movement_vector = np.array(msg.data)
        if len(movement_vector) != self.robot.get_joints_number():
            self.get_logger().error('The movement vector has a different size than the number of joints')
            return
        self.robot.compute_fk(movement_vector)
        self.tf_messages = [TransformStamped() for joint in range(self.robot.get_joints_number())]
        for idx, tf_message in enumerate(self.tf_messages):
            tf_message.transform.translation.x = self.robot.get_joint_transforms()[idx][0][3]
            tf_message.transform.translation.y = self.robot.get_joint_transforms()[idx][1][3]
            tf_message.transform.translation.z = self.robot.get_joint_transforms()[idx][2][3]
            tf_message.transform.rotation.x = quaternion_from_matrix(self.robot.get_joint_transforms()[idx])[1]
            tf_message.transform.rotation.y = quaternion_from_matrix(self.robot.get_joint_transforms()[idx])[2]
            tf_message.transform.rotation.z = quaternion_from_matrix(self.robot.get_joint_transforms()[idx])[3]
            tf_message.transform.rotation.w = quaternion_from_matrix(self.robot.get_joint_transforms()[idx])[0]
            if idx == 0:
                tf_message.header.frame_id = 'world'
            else:
                tf_message.header.frame_id = f'joint_{idx-1}'
            tf_message.child_frame_id = f'joint_{idx}'
            self.tf_broadcaster.sendTransform(tf_message)

        self.get_logger().info('TF messages have been sent')
        self.get_logger().info(f'End effector position: {self.robot.end_effector}')

def main():
    rclpy.init() 
    robot = Robot(RevoluteJoint(), RevoluteJoint(l_offset=1))
    variable_robot_kinematics = VariableRobotKinematics(robot)
    rclpy.spin(variable_robot_kinematics)
    variable_robot_kinematics.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()