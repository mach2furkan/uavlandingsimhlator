#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionProcessor(Node):
    def __init__(self):
        super().__init__('vision_processor')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/landing_target/pose', 10)
        self.debug_image_publisher = self.create_publisher(Image, '/vision/debug_output', 10)
        
        self.bridge = CvBridge()
        
        # ArUco Configuration
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        try:
            self.aruco_params = cv2.aruco.DetectorParameters()
        except AttributeError:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # Camera Matrix (Simulated for 640x480, 60deg FOV approx)
        # fx = width / (2 * tan(fov/2))
        # fov_h = 1.047 rad (60 deg) -> tan(30) = 0.577
        # fx = 640 / (2 * 0.577) ~= 554
        self.camera_matrix = np.array([[554, 0, 320], [0, 554, 240], [0, 0, 1]], dtype=float)
        self.dist_coeffs = np.zeros((5, 1))

        self.get_logger().info('Vision Processor Node Started')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            # Estimate pose (using 1.0 meter marker size)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 1.0, self.camera_matrix, self.dist_coeffs)
            
            for i in range(len(ids)):
                # Publish the first marker found
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'camera_link'
                
                # Translation
                pose_msg.pose.position.x = tvecs[i][0][0]
                pose_msg.pose.position.y = tvecs[i][0][1]
                pose_msg.pose.position.z = tvecs[i][0][2]
                
                # Rotation (Convert Rodrigues to Quat - simplified for now, just sending identity or keeping as is if we had conversions helper)
                # For this demo, we mainly care about Position X/Y/Z for alignment
                pose_msg.pose.orientation.w = 1.0
                
                self.pose_publisher.publish(pose_msg)
                
                # Draw axis
                cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.5)
            
            # Draw markers
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
        
        # Publish Debug Image
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.debug_image_publisher.publish(debug_msg)
        except Exception as e:
             self.get_logger().error(f'Failed to publish debug image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = VisionProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
