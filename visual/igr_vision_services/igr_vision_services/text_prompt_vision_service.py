#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from vision_interfaces.srv import GetFeatures
from igr_vision_services.text_prompt_vision import TextPromptVision

import numpy as np

class TextPromptVisionNode(Node):
    def __init__(self):
        super().__init__('text_prompt_vision_node')
        self.text_prompt_vision = TextPromptVision()
        self.srv = self.create_service(GetFeatures, 'get_features', self.get_features_callback)
    def get_features_callback(self, request, response):
        # Convert the ROS Image message to a NumPy array
        image_np = np.array(request.image.data, dtype=np.uint8).reshape(request.image.height, request.image.width, -1)

        # Call the detect and segment methods
        boxes, pred_phrases = self.text_prompt_vision.detect([image_np], [[request.text_prompt]])
        # masks, binary_masks, labels = self.text_prompt_vision.segment(image_np, boxes[0], pred_phrases[0])
        masks = self.text_prompt_vision.segment(image_np, boxes[0], pred_phrases[0])
        # self.text_prompt_vision.draw_segmentation(image_np, boxes[0], masks, pred_phrases[0])
        
        # **** TEST ONLY ****
        self.text_prompt_vision.draw_segmentation(image_np, [boxes[0][0]], [masks[0]], [pred_phrases[0][0]])

        # Get the features
        # centroids, principal_axes = self.text_prompt_vision.get_features(request.text_prompt, binary_masks, labels)
        centroids, principal_axes = self.text_prompt_vision.get_features(request.text_prompt)
        # self.text_prompt_vision.draw_features(image_np, centroids, principal_axes)

        # **** TEST ONLY ****
        self.text_prompt_vision.draw_features(image_np, [centroids[0]], [principal_axes[0]])

        # Fill the response with the features
        for centroid, principal_axis in zip(centroids, principal_axes):
            response.centroid_x.append(centroid[0])
            response.centroid_y.append(centroid[1])
            response.principal_axis_x.append(principal_axis[0])
            response.principal_axis_y.append(principal_axis[1])
        self.get_logger().info("Done feature extraction service call.")
        return response

def main(args=None):
    rclpy.init(args=args)

    text_prompt_vision_node = TextPromptVisionNode()

    # Start the service
    text_prompt_vision_node.get_logger().info("Feature extraction service is running.")
    executor = MultiThreadedExecutor()
    rclpy.spin(text_prompt_vision_node, executor=executor)

    # Cleanup
    rclpy.shutdown()


if __name__ == '__main__':
    main()