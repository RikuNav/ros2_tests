import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D

class PathGenerator(Node):
    def __init__(self):
        super().__init__('generate_path')
        #### SUBSCRIBERS ####
        self.vertices_subscriber = self.create_subscription(Float32, 'vertices', self.vertices_callback, 10)
        self.ready_subscriber = self.create_subscription(Float32, 'ready', self.ready_callback, 10)
        #### PUBLISHERS ####
        self.point_publisher = self.create_publisher(Pose2D, 'point', 10)
        self.enable_path = False
        self.num_vertices = 0
        self.vertices_idx = -1
        self.points = []
        self.pose = Pose2D()
        self.get_logger().info('Path Generator Node has been started!')

    def ready_callback(self, msg):
        if self.enable_path:
            self.update_pose()
            self.point_publisher.publish(self.pose)
            self.get_logger().info('Path has been generated!')

    def vertices_callback(self, msg):
        self.points = self.generate_path(int(msg.data))
        self.update_pose()
        self.point_publisher.publish(self.pose)
        self.enable_path = True

    def generate_path(self, vertices):
        self.num_vertices = vertices
        self.vertices_idx = -1
        angles = 2 * np.pi / vertices
        self.points = []
        for i in range(vertices):
            self.points.append((np.cos(angles * i)-1.0, np.sin(angles * i)))
        self.points.append(self.points[0])
        return self.points

    def update_pose(self):
        self.vertices_idx += 1
        self.pose.x = self.points[self.vertices_idx][0]
        self.pose.y = self.points[self.vertices_idx][1]
        self.pose.theta = 0.0
        if self.vertices_idx == self.num_vertices:
            self.enable_path = False
            
def main():
    rclpy.init()
    path_generator = PathGenerator()
    rclpy.spin(path_generator)
    path_generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()