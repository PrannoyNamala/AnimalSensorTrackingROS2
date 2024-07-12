#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import GetModelState

class PositionPublisher(Node):

    def __init__(self):
        super().__init__('position_publisher')
        self.publisher_ = self.create_publisher(Pose, 'floating_object_position', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.cli = self.create_client(GetModelState, 'get_model_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetModelState.Request()
        self.req.model_name = 'floating_object'

    def timer_callback(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result() is not None:
            pose = self.future.result().pose
            self.publisher_.publish(pose)
            self.get_logger().info('Publishing: "%s"' % pose)
        else:
            self.get_logger().error('Exception while calling service: %r' % self.future.exception())

def main(args=None):
    rclpy.init(args=args)
    position_publisher = PositionPublisher()
    rclpy.spin(position_publisher)
    position_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

