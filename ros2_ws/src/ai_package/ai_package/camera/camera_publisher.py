#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # Parameters
        self.declare_parameter('device', 0)
        self.declare_parameter('topic_name', '/camera/image_raw')
        
        self.device = self.get_parameter('device').value
        self.topic_name = self.get_parameter('topic_name').value
        
        # Initialize camera
        self.cap = cv2.VideoCapture(self.device)
        
        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open camera: {self.device}')
            raise RuntimeError(f'Cannot open camera: {self.device}')
        
        # Get camera's native resolution and FPS
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        
        # If FPS is invalid, use default
        if self.fps <= 0:
            self.fps = 30
            self.get_logger().warn(f'Invalid FPS detected, using default: {self.fps}')
        
        # ROS 2 components
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, self.topic_name, 10)
        self.timer = self.create_timer(1.0 / self.fps, self.timer_callback)
        
        self.get_logger().info(f'Camera publisher started: {self.device}')
        self.get_logger().info(f'Camera resolution: {self.frame_width}x{self.frame_height}')
        self.get_logger().info(f'Camera FPS: {self.fps}')
        self.get_logger().info(f'Publishing to: {self.topic_name}')
        
    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().warn('Failed to read frame from camera')
            return
        
        try:
            # Convert to ROS message
            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "camera_frame"
            
            # Publish
            self.publisher.publish(ros_image)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing image: {e}')
    
    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        self.get_logger().info('Camera publisher stopped')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CameraPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
