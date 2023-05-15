#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from vision_interfaces.srv import GetStereo
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Point
from tf2_geometry_msgs import do_transform_point
import numpy as np
import time
import tf2_ros
from tf2_ros import TransformListener, Buffer
from igr_vision_services.text_prompt_vision import TextPromptVision

class StereoVisionServiceNode(Node):
    def __init__(self):
        super().__init__('stereo_vision_service_node')
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('RGB_image_topic', '/rgbd_camera/image'),
                ('depth_image_topic', '/rgbd_camera/depth_image'),
                ('camera_optical_frame', 'RGBD_optical_link'),
                ('reference_frame', 'world'),
                ('simulation_on', True),
                ('camera_intrinsic.cx', 480.0),
                ('camera_intrinsic.cy', 360.0),
                ('camera_intrinsic.fx', 831.57),
                ('camera_intrinsic.fy', 831.57),
            ]
        )
        # load params
        self.RGB_image_topic = self.get_parameter('RGB_image_topic').value
        self.depth_image_topic = self.get_parameter('depth_image_topic').value
        self.camera_optical_frame = self.get_parameter('camera_optical_frame').value
        self.reference_frame = self.get_parameter('reference_frame').value
        self.simulation_on = self.get_parameter('simulation_on').value
        self.cx = self.get_parameter('camera_intrinsic.cx').value
        self.cy = self.get_parameter('camera_intrinsic.cy').value
        self.fx = self.get_parameter('camera_intrinsic.fx').value
        self.fy = self.get_parameter('camera_intrinsic.fy').value
        # images
        self.rgb_image = None
        self.depth_image = None
        self.rgb_image_last = None
        self.depth_image_last = None
        self.camera_transform_last = None
        # transform setups
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.srv = self.create_service(GetStereo, 'get_stereo', self.get_stereo_callback)
        self.sub_rgb = self.create_subscription(Image, self.RGB_image_topic, self.rgb_callback, 100)
        self.sub_depth = self.create_subscription(Image, self.depth_image_topic, self.depth_callback, 100)

        self.feature_extractor = TextPromptVision()
    
    def rgb_callback(self, msg):
        # Convert the ROS Image message to a NumPy array
        self.rgb_image = np.array(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    
    def depth_callback(self, msg):
        # Convert the ROS Image message to a NumPy array
        depth_image = np.zeros((msg.height, msg.width), dtype=np.float32)
        depth_raw = np.array(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        depth_image = depth_raw.view(np.float32)
        self.depth_image = depth_image.copy()


    def feature_extraction(self, text_prompt):
        self.get_logger().info("Waiting for image ...")
        while self.rgb_image is None:
            rclpy.spin_once(self, timeout_sec=0.01)
        while self.depth_image is None:
            rclpy.spin_once(self, timeout_sec=0.01)
        self.get_logger().info("Extracting camera transform ...")
        # self.camera_transform_last = self.get_transform(self.camera_optical_frame, self.reference_frame, timeout_sec=2.0)
        self.camera_transform_last = self.get_transform(self.reference_frame, self.camera_optical_frame, timeout_sec=2.0)
        self.rgb_image_last = self.rgb_image
        self.depth_image_last = self.depth_image
        self.get_logger().info("Transform and images received.")

        # Call the detect and segment methods
        boxes, pred_phrases = self.feature_extractor.detect([self.rgb_image_last], [[text_prompt]])
        masks = self.feature_extractor.segment(self.rgb_image_last, boxes[0], pred_phrases[0])
        # self.feature_extractor.draw_segmentation(image_np, boxes[0], masks, pred_phrases[0])
        
        # **** TEST ONLY ****
        self.feature_extractor.draw_segmentation(self.rgb_image_last, [boxes[0][0]], [masks[0]], [pred_phrases[0][0]])

        # Get the features
        # centroids, principal_axes = self.feature_extractor.get_features(request.text_prompt, binary_masks, labels)
        centroids, principal_axes = self.feature_extractor.get_features(text_prompt)
        # self.feature_extractor.draw_features(image_np, centroids, principal_axes)

        # **** TEST ONLY ****
        self.feature_extractor.draw_features(self.rgb_image_last, [centroids[0]], [principal_axes[0]])

        centroid_u, centroid_v = [], []
        # Fill the response with the features
        for centroid, _ in zip(centroids, principal_axes):
            centroid_u.append(centroid[0])
            centroid_v.append(centroid[1])

        return centroid_u, centroid_v

    def get_stereo_callback(self, request, response):
        text_prompt = request.text_prompt
        centroid_u, centroid_v = self.feature_extraction(text_prompt)
        self.get_logger().info("Features obtained.")
        positions_x, positions_y, positions_z = self.get_positions(centroid_u, centroid_v)
        self.get_logger().info("Positions obtained.")
        positions = [[float(x), float(y), float(z)] for x, y, z in zip(positions_x, positions_y, positions_z)]
        self.get_logger().info("Tranforming positions ...")
        for position in positions:
            point_msg = Point(x=position[0], y=position[1], z=position[2])
            point_camera_frame_stamped = PointStamped()
            point_camera_frame_stamped.header.frame_id = "camera_frame"
            point_camera_frame_stamped.point = point_msg 
            point_world_frame_stamped = do_transform_point(point_camera_frame_stamped, self.camera_transform_last)
            response.positions_x.append(point_world_frame_stamped.point.x)
            response.positions_y.append(point_world_frame_stamped.point.y)
            response.positions_z.append(point_world_frame_stamped.point.z)
        self.get_logger().info("Positions tranformed.")
        return response
    
    def get_positions(self, centroid_u, centroid_v):
        positions_x, positions_y, positions_z = [], [], []
        for u, v in zip(centroid_u, centroid_v):
            depth = self.depth_image_last[int(v), int(u)]
            if np.isnan(depth) or np.isinf(depth):
                positions_x.append(np.nan)
                positions_y.append(np.nan)
                positions_z.append(np.nan)
            else:
                x = (u - self.cx) * depth / self.fx
                y = (v - self.cy) * depth / self.fy
                z = depth
                positions_x.append(x)
                positions_y.append(y)
                positions_z.append(z)
        return positions_x, positions_y, positions_z
    
    def image_msg_from_numpy(self, image_np):
        image_msg = Image()
        image_msg.height, image_msg.width, _ = image_np.shape
        image_msg.encoding = "rgb8"
        image_msg.step = image_np.shape[1] * 3
        image_msg.data = image_np.flatten().tolist()
        return image_msg
    
    def get_transform(self, target_frame, source_frame, timeout_sec=2.0):
        start_time = time.time()
        while True:
            try:
                transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
                return transform
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                if time.time() - start_time > timeout_sec:
                    self.get_logger().error('Timeout exceeded while waiting for transform')
                    return None
                self.get_logger().info('Transform not available yet. Retrying...')


def main(args=None):
    rclpy.init(args=args)

    stereo_vision_service_node = StereoVisionServiceNode()

    # Start the service
    stereo_vision_service_node.get_logger().info("Stereo vision service is running.")
    executor = MultiThreadedExecutor()
    rclpy.spin(stereo_vision_service_node, executor=executor)

    # Cleanup
    rclpy.shutdown()


if __name__ == '__main__':
    main()