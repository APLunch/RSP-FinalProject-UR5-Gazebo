#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vision_interfaces.srv import GetStereo
import geometry_msgs.msg
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class StereoVisionTestClient(Node):
    def __init__(self):
        super().__init__("text_prompt_vision_test_client")

        self.declare_parameter('text_prompt', 'blue ball')
        self.text_prompt = self.get_parameter('text_prompt').value

        self.br = StaticTransformBroadcaster(self)
        self.cli = self.create_client(GetStereo, "get_stereo")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for 'get_stereo' service to become available...")

    def call_service(self):
        # Create a request
        request = GetStereo.Request()
        request.text_prompt = self.text_prompt

        # Call the service
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error("Failed to call the service.")
            return None
    
    def publish_transform(self, translation, rotation, parent_frame_id, child_frame_id):
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame_id
        t.child_frame_id = child_frame_id
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        t.transform.rotation.x = rotation[0]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = rotation[3]

        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    stereo_vision_test_client = StereoVisionTestClient()

    # Call the service
    response = stereo_vision_test_client.call_service()

    if response:
        positions_x = response.positions_x
        positions_y = response.positions_y
        positions_z = response.positions_z
        print(list(positions_x))
        print(list(positions_y))
        print(list(positions_z))
        for i, (x, y, z) in enumerate(zip(positions_x, positions_y, positions_z)):
            stereo_vision_test_client.publish_transform([x, y, z], [0.0, 0.0, 0.0, 1.0], "world", f"target{i}")

    rclpy.spin(stereo_vision_test_client)

    stereo_vision_test_client.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()