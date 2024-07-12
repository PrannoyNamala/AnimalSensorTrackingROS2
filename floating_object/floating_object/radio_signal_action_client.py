#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point
from radio_signal_interface.action import RadioSignal

class RadioSignalActionClient(Node):

    def __init__(self):
        super().__init__('radio_signal_action_client')
        self._action_client = ActionClient(self, RadioSignal, 'radio_signal')

    def send_goal(self, x, y, z):
        goal_msg = RadioSignal.Goal()
        goal_msg.object_position = Point(x=x, y=y, z=z)

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: Angle={result.angle}, Distance={result.distance}')

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback: {feedback_msg.feedback}')

def main(args=None):
    rclpy.init(args=args)
    node = RadioSignalActionClient()
    node.send_goal(2.0, 2.0, 0.0)
    rclpy.spin(node)

if __name__ == '__main__':
    main()

