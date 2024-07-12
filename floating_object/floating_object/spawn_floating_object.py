#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
import os
from ament_index_python.packages import get_package_share_directory


class SpawnFloatingObject(Node):

    def __init__(self):
        super().__init__('spawn_floating_object')
        self.cli = self.create_client(SpawnEntity, 'spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SpawnEntity.Request()

    def send_request(self):
        urdf_file_path = os.path.join(
            get_package_share_directory('floating_object'), 'urdf', 'floating_object.urdf')
        with open(urdf_file_path, 'r') as urdf_file:
            self.req.name = 'floating_object'
            self.req.xml = urdf_file.read()
            self.req.robot_namespace = ''
            self.req.reference_frame = 'world'
            self.req.initial_pose.position.x = 0.0
            self.req.initial_pose.position.y = 0.0
            self.req.initial_pose.position.z = 0.5  # Adjust height as needed

        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    spawn_floating_object = SpawnFloatingObject()
    spawn_floating_object.send_request()

    while rclpy.ok():
        rclpy.spin_once(spawn_floating_object)
        if spawn_floating_object.future.done():
            try:
                response = spawn_floating_object.future.result()
            except Exception as e:
                spawn_floating_object.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                spawn_floating_object.get_logger().info(
                    'Successfully spawned entity: %s' % response.success)
            break

    spawn_floating_object.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

