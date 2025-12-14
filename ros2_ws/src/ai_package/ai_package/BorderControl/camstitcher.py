#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import numpy as np
import time
import os
from datetime import datetime
from cv_bridge import CvBridge

class PanoramaCreator(Node):
    def __init__(self):
        super().__init__('panorama_creator')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('camera_topic', '/camera/image_raw'),
                ('frame_interval', 10),
                ('roi_expansion', 150),
                ('output_path', 'panorama_output.jpg'),
                ('publish_progress', True),
                ('max_panorama_width', 5000),  # Prevent excessive memory usage
                ('min_matches', 8),
                ('feature_detector', 'SIFT'),
            ]
        )
        
        # Get parameters
        self.frame_interval = self.get_parameter('frame_interval').value
        self.roi_expansion = self.get_parameter('roi_expansion').value
        self.output_path = self.get_parameter('output_path').value
        self.publish_progress = self.get_parameter('publish_progress').value
        self.max_panorama_width = self.get_parameter('max_panorama_width').value
        self.min_matches = self.get_parameter('min_matches').value
        
        # Initialize variables
        self.panorama = None
        self.gray_panorama = None
        self.last_h_adjusted = None
        self.frame_count = 0
        self.start_time = time.time()
        self.bridge = CvBridge()
        self.is_active = True
        
        # Check CUDA availability
        self.use_cuda = cv2.cuda.getCudaEnabledDeviceCount() > 0
        self.get_logger().info(f"CUDA available: {self.use_cuda}")
        
        # Initialize feature detector based on parameter
        detector_type = self.get_parameter('feature_detector').value
        if detector_type == 'SIFT':
            self.sift = cv2.SIFT_create(
                nfeatures=17000,
                nOctaveLayers=3,
                contrastThreshold=0.04,
                edgeThreshold=10
            )
        elif detector_type == 'ORB':
            self.sift = cv2.ORB_create(nfeatures=10000)
        else:
            self.get_logger().warn(f"Unknown detector {detector_type}, using SIFT")
            self.sift = cv2.SIFT_create(nfeatures=17000)
        
        # FLANN matcher
        if detector_type == 'SIFT':
            self.flann = cv2.FlannBasedMatcher(
                dict(algorithm=1, trees=3),
                dict(checks=30)
            )
        else:  # ORB uses different parameters
            self.flann = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        
        # Pre-allocate memory for corners
        self.corners_template = np.array([[0, 0], [0, 1], [1, 1], [1, 0]], dtype=np.float32)
        
        # Publishers
        self.panorama_pub = self.create_publisher(Image, 'panorama_progress', 10)
        self.status_pub = self.create_publisher(String, 'panorama_status', 10)
        
        # Subscriber
        camera_topic = self.get_parameter('camera_topic').value
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            qos_profile
        )
        
        # Service for starting/stopping (could be added later)
        # self.service = self.create_service(Trigger, 'toggle_panorama', self.toggle_callback)
        
        # Timer for status updates
        self.status_timer = self.create_timer(2.0, self.publish_status)
        
        self.get_logger().info("Panorama creator initialized and ready")
        self.get_logger().info(f"Subscribed to: {camera_topic}")
        self.get_logger().info("Send empty string to 'panorama_status' topic to save and exit")
    
    def image_callback(self, msg):
        """Process incoming camera images"""
        if not self.is_active:
            return
            
        try:
            # Convert ROS Image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {str(e)}")
            return
        
        self.frame_count += 1
        
        # Skip frames based on interval
        if self.frame_count % self.frame_interval != 0:
            return
        
        self.get_logger().info(f"Processing frame {self.frame_count}", throttle_duration_sec=5.0)
        
        # Convert to grayscale
        if self.use_cuda:
            gpu_frame = cv2.cuda_GpuMat()
            gpu_frame.upload(frame)
            gray_frame = cv2.cuda.cvtColor(gpu_frame, cv2.COLOR_BGR2GRAY).download()
        else:
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Initialize panorama if first frame
        if self.panorama is None:
            self.panorama = frame.copy()
            self.gray_panorama = gray_frame.copy()
            
            # Initialize corners template with actual frame dimensions
            h, w = frame.shape[:2]
            self.corners_template = np.array([[0, 0], [0, h], [w, h], [w, 0]], dtype=np.float32)
            
            self.publish_panorama_progress()
            return
        
        # ROI processing
        if self.last_h_adjusted is not None:
            warped_corners = cv2.perspectiveTransform(
                self.corners_template.reshape(-1, 1, 2), 
                self.last_h_adjusted
            ).reshape(-1, 2)
            
            x_min, y_min = warped_corners.min(axis=0).astype(int) - self.roi_expansion
            x_max, y_max = warped_corners.max(axis=0).astype(int) + self.roi_expansion
            
            x_min = max(0, x_min)
            y_min = max(0, y_min)
            x_max = min(self.panorama.shape[1], x_max)
            y_max = min(self.panorama.shape[0], y_max)
            
            if x_max > x_min and y_max > y_min:
                roi = self.gray_panorama[y_min:y_max, x_min:x_max]
                kp_pano, des_pano = self.sift.detectAndCompute(roi, None)
                
                if des_pano is not None:
                    for kp in kp_pano:
                        kp.pt = (kp.pt[0] + x_min, kp.pt[1] + y_min)
            else:
                kp_pano, des_pano = None, None
        else:
            kp_pano, des_pano = self.sift.detectAndCompute(self.gray_panorama, None)
        
        kp_frame, des_frame = self.sift.detectAndCompute(gray_frame, None)
        
        # Check keypoints
        if (des_pano is None or des_frame is None or 
            len(des_pano) < 10 or len(des_frame) < 10):
            self.get_logger().warn(f"Frame {self.frame_count}: insufficient keypoints")
            return
        
        # Feature matching
        if hasattr(self.flann, 'knnMatch'):
            matches = self.flann.knnMatch(des_pano, des_frame, k=2)
            good_matches = [m for m, n in matches if m.distance < 0.75 * n.distance]
        else:
            matches = self.flann.match(des_pano, des_frame)
            matches = sorted(matches, key=lambda x: x.distance)
            good_matches = matches[:min(50, len(matches))]
        
        if len(good_matches) < self.min_matches:
            self.get_logger().warn(f"Frame {self.frame_count}: insufficient matches ({len(good_matches)})")
            return
        
        # Calculate homography
        src_pts = np.float32([kp_pano[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp_frame[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        
        H, mask = cv2.findHomography(dst_pts, src_pts, cv2.RANSAC, 3.0)
        
        if H is None:
            self.get_logger().warn(f"Frame {self.frame_count}: homography failed")
            return
        
        # Calculate panorama boundaries
        h, w = frame.shape[:2]
        frame_corners = np.array([[0, 0], [0, h], [w, h], [w, 0]], dtype=np.float32)
        warped_corners = cv2.perspectiveTransform(frame_corners.reshape(-1, 1, 2), H)
        
        x_min = int(min(0, warped_corners[:, :, 0].min()))
        y_min = int(min(0, warped_corners[:, :, 1].min()))
        x_max = int(max(self.panorama.shape[1], warped_corners[:, :, 0].max()))
        y_max = int(max(self.panorama.shape[0], warped_corners[:, :, 1].max()))
        
        # Check panorama size limits
        if (x_max - x_min) > self.max_panorama_width:
            self.get_logger().warn("Panorama too wide, skipping frame")
            return
        
        # Adjust homography for translation
        translation = np.array([[1, 0, -x_min], [0, 1, -y_min], [0, 0, 1]])
        H_adjusted = translation @ H
        self.last_h_adjusted = H_adjusted
        
        # Warp frame
        if self.use_cuda:
            gpu_frame.upload(frame)
            gpu_warped = cv2.cuda.warpPerspective(
                gpu_frame, 
                H_adjusted, 
                (int(x_max - x_min), int(y_max - y_min)))
            warped_frame = gpu_warped.download()
        else:
            warped_frame = cv2.warpPerspective(
                frame, 
                H_adjusted, 
                (int(x_max - x_min), int(y_max - y_min)))
        
        # Blend with existing panorama
        mask = (warped_frame[..., 0] != 0) | (warped_frame[..., 1] != 0) | (warped_frame[..., 2] != 0)
        panorama_adjusted = np.zeros_like(warped_frame)
        panorama_adjusted[-y_min:self.panorama.shape[0]-y_min, -x_min:self.panorama.shape[1]-x_min] = self.panorama
        
        self.panorama = np.where(mask[..., None], warped_frame, panorama_adjusted)
        self.gray_panorama = cv2.cvtColor(self.panorama, cv2.COLOR_BGR2GRAY)
        
        # Publish progress
        self.publish_panorama_progress()
    
    def publish_panorama_progress(self):
        """Publish current panorama state"""
        if self.panorama is None or not self.publish_progress:
            return
            
        try:
            # Create display version with info
            display_pano = self.panorama.copy()
            
            # Add progress information
            cv2.putText(display_pano, f"Frames: {self.frame_count}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(display_pano, f"Size: {display_pano.shape[1]}x{display_pano.shape[0]}", 
                       (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Convert to ROS message and publish
            panorama_msg = self.bridge.cv2_to_imgmsg(display_pano, encoding="bgr8")
            panorama_msg.header.stamp = self.get_clock().now().to_msg()
            panorama_msg.header.frame_id = "panorama"
            
            self.panorama_pub.publish(panorama_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing panorama: {str(e)}")
    
    def publish_status(self):
        """Publish status information"""
        status_msg = String()
        if self.panorama is None:
            status_msg.data = "Waiting for first frame..."
        else:
            elapsed = time.time() - self.start_time
            fps = self.frame_count / elapsed if elapsed > 0 else 0
            status_msg.data = f"Processing: {self.frame_count} frames, {fps:.1f} FPS, Size: {self.panorama.shape[1]}x{self.panorama.shape[0]}"
        
        self.status_pub.publish(status_msg)
    
    def save_panorama(self):
        """Save the final panorama and cleanup"""
        if self.panorama is None:
            self.get_logger().warn("No panorama to save")
            return False
        
        try:
            # Crop black borders
            gray = cv2.cvtColor(self.panorama, cv2.COLOR_BGR2GRAY)
            _, thresh = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                x, y, w, h = cv2.boundingRect(max(contours, key=cv2.contourArea))
                panorama_cropped = self.panorama[y:y+h, x:x+w]
            else:
                panorama_cropped = self.panorama
            
            # Save panorama
            cv2.imwrite(self.output_path, panorama_cropped)
            
            # Log statistics
            elapsed_time = time.time() - self.start_time
            self.get_logger().info(f"Panorama saved to: {self.output_path}")
            self.get_logger().info(f"Total frames processed: {self.frame_count}")
            self.get_logger().info(f"Total time: {elapsed_time:.2f} sec")
            self.get_logger().info(f"Processing speed: {self.frame_count/elapsed_time:.2f} FPS")
            self.get_logger().info(f"Final panorama size: {panorama_cropped.shape[1]}x{panorama_cropped.shape[0]}")
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error saving panorama: {str(e)}")
            return False
    
    def shutdown(self):
        """Clean shutdown"""
        self.is_active = False
        self.save_panorama()

def main(args=None):
    rclpy.init(args=args)
    
    panorama_creator = PanoramaCreator()
    
    try:
        rclpy.spin(panorama_creator)
    except KeyboardInterrupt:
        panorama_creator.get_logger().info("Keyboard interrupt received")
    finally:
        panorama_creator.shutdown()
        panorama_creator.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
