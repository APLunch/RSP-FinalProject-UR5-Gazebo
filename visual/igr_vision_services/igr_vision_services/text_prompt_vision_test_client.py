#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vision_interfaces.srv import GetFeatures
from sensor_msgs.msg import Image
import numpy as np

class TextPromptVisionTestClient(Node):
    def __init__(self):
        super().__init__("text_prompt_vision_test_client")
        self.declare_parameter('image_topic', '/image1')
        self.declare_parameter('image_topic2', '/image2')
        self.declare_parameter('text_prompt', 'red ball')

        self.image_topic = self.get_parameter('image_topic').value
        self.image_topic2 = self.get_parameter('image_topic2').value
        self.text_prompt = self.get_parameter('text_prompt').value

        self.cli = self.create_client(GetFeatures, "get_features")
        self.sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 100)
        self.latest_image_np = None
        if self.image_topic2 is not None:
            self.sub2 = self.create_subscription(
                Image, self.image_topic2, self.image_callback2, 100)
            self.latest_image_np2 = None
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for 'get_features' service to become available...")

    def image_callback(self, msg):
        # Convert the ROS Image message to a NumPy array
        self.latest_image_np = np.array(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    
    def image_callback2(self, msg):
        # Convert the ROS Image message to a NumPy array
        self.latest_image_np2 = np.array(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)

    def call_service(self):
        # Convert the NumPy array to a ROS Image message
        self.get_logger().info("Waiting for image ...")

        while self.latest_image_np is None:
            rclpy.spin_once(self, timeout_sec=0.01)
        image = self.latest_image_np
    
        self.get_logger().info("Image received.")

        # Create a request
        request = GetFeatures.Request()
        request.image = self.image_msg_from_numpy(image)
        request.text_prompt = self.text_prompt

        # Call the service
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error("Failed to call the service.")
            return None
        
    def image_msg_from_numpy(self, image_np):
        image_msg = Image()
        image_msg.height, image_msg.width, _ = image_np.shape
        image_msg.encoding = "rgb8"
        image_msg.step = image_np.shape[1] * 3
        image_msg.data = image_np.flatten().tolist()
        return image_msg

def main(args=None):
    rclpy.init(args=args)

    text_prompt_vision_client = TextPromptVisionTestClient()

    # Call the service
    response = text_prompt_vision_client.call_service()

    if response:
        print("Service response:")
        print("Centroid X:", response.centroid_x)
        print("Centroid Y:", response.centroid_y)
        print("Principal axis X:", response.principal_axis_x)
        print("Principal axis Y:", response.principal_axis_y)

    # Cleanup
    rclpy.shutdown()

if __name__ == "__main__":
    main()