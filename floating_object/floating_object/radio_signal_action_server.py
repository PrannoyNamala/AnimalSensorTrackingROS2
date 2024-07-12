#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Point
from radio_signal_interface.action import RadioSignal
from math import atan2, sqrt

class RadioSignalActionServer(Node):

    def __init__(self):
        super().__init__('radio_signal_action_server')
        self._action_server = ActionServer(
            self,
            RadioSignal,
            'radio_signal',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        object_position = goal_handle.request.object_position
        # WRITE CODE TO UPDATE THE POSITION OF SENSOR
        floating_object_position = Point(x=5.0, y=5.0, z=0.0) 

        # Calculate distance
        distance = sqrt(
            (floating_object_position.x - object_position.x) ** 2 +
            (floating_object_position.y - object_position.y) ** 2 +
            (floating_object_position.z - object_position.z) ** 2
        )

        # Calculate angle (in radians)
        angle = atan2(
            object_position.y - floating_object_position.y,
            object_position.x - floating_object_position.x
        )

        goal_handle.succeed()

        result = RadioSignal.Result()
        result.angle = angle
        result.distance = distance if distance < 10.0 else float('nan')  # Sending distance only if close

        return result

def main(args=None):
    rclpy.init(args=args)
    node = RadioSignalActionServer()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

