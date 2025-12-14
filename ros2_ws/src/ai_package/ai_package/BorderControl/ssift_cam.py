#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
import cv2
import numpy as np
from collections import deque
import time
import csv
import os
from datetime import datetime
from cv_bridge import CvBridge
import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect

class OptimizedDroneTracker(Node):
    def __init__(self, panorama_path, frame_interval=5, max_history=100):
        super().__init__('drone_tracker')
        
        # Parameters
        self.declare_parameter('mavlink_enabled', True)
        self.declare_parameter('mavlink_port', '/dev/ttyACM0')
        self.declare_parameter('position_update_interval', 1.0)  # Seconds between MAVLink updates
        
        self.mavlink_enabled = self.get_parameter('mavlink_enabled').value
        self.mavlink_port = self.get_parameter('mavlink_port').value
        self.position_update_interval = self.get_parameter('position_update_interval').value
        
        # Load panorama image
        self.panorama = cv2.imread(panorama_path)
        if self.panorama is None:
            self.get_logger().error(f"Не удалось загрузить панораму: {panorama_path}")
            raise ValueError(f"Не удалось загрузить панораму: {panorama_path}")
        
        self.frame_interval = max(1, frame_interval)
        self.frame_counter = 0
        self.max_history = max_history
        
        # Initialize MAVLink connection
        self.mavlink_connection = None
        self.last_mavlink_update_time = 0
        if self.mavlink_enabled:
            self.setup_mavlink_connection()
        
        # Initialize feature detectors and matcher
        self.detector_0 = cv2.ORB_create(10000)
        self.detector_1 = cv2.ORB_create(nfeatures=5000)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        
        # Extract features from panorama
        gray_pano = cv2.cvtColor(self.panorama, cv2.COLOR_BGR2GRAY)
        self.pano_kp, self.pano_des = self.detector_0.detectAndCompute(gray_pano, None)
        
        self.positions = deque(maxlen=max_history)
        self.last_position = None
        self.bridge = CvBridge()
        
        # ROS2 publishers and subscribers
        self.position_pub = self.create_publisher(PointStamped, 'drone_position', 10)
        self.visualization_pub = self.create_publisher(Image, 'tracking_visualization', 10)
        self.mavlink_status_pub = self.create_publisher(Header, 'mavlink_status', 10)
        
        # Subscribe to camera topic
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )
        
        # Create a timer for periodic MAVLink status updates
        self.status_timer = self.create_timer(5.0, self.publish_mavlink_status)
        
        self.init_log_files()
        self.get_logger().info("Drone tracker initialized and ready")
        self.get_logger().info(f"MAVLink enabled: {self.mavlink_enabled}")
    
    def setup_mavlink_connection(self):
        """Настраивает соединение MAVLink с дроном"""
        try:
            self.get_logger().info(f"Connecting to MAVLink on port {self.mavlink_port}...")
            self.mavlink_connection = utility.mavlink_connection(
                device=self.mavlink_port,
                source_system=1,
                source_component=0
            )
            
            # Wait for heartbeat with timeout
            self.mavlink_connection.wait_heartbeat(timeout=5)
            self.get_logger().info(f"MAVLink connected! System: {self.mavlink_connection.target_system}, Component: {self.mavlink_connection.target_component}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MAVLink: {e}")
            self.mavlink_connection = None
    
    def send_mavlink_position(self, position, timestamp=None):
        """
        Отправляет позицию дрона через MAVLink
        
        Args:
            position: (x, y) координаты на панораме
            timestamp: Временная метка
        """
        if self.mavlink_connection is None:
            return False
        
        current_time = time.time()
        if current_time - self.last_mavlink_update_time < self.position_update_interval:
            return False
        
        try:
            # Send position via STATUSTEXT
            message_text = f"DRONE_POS: X={position[0]:.1f}, Y={position[1]:.1f}"
            message = dialect.MAVLink_statustext_message(
                severity=dialect.MAV_SEVERITY_INFO,
                text=message_text.encode("utf-8")
            )
            self.mavlink_connection.mav.send(message)
            
            # Also send as GLOBAL_POSITION_INT (simplified for 2D)
            msg = dialect.MAVLink_global_position_int_message(
                time_boot_ms=int(current_time * 1000),
                lat=int(position[0] * 1e7),  # Convert to degrees * 1e7
                lon=int(position[1] * 1e7),  # Convert to degrees * 1e7
                alt=0,
                relative_alt=0,
                vx=0,
                vy=0,
                vz=0,
                hdg=0
            )
            self.mavlink_connection.mav.send(msg)
            
            self.last_mavlink_update_time = current_time
            self.get_logger().info(f"MAVLink position sent: {message_text}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error sending MAVLink position: {e}")
            return False
    
    def send_statustext_to_gcs(self, message_text, severity=dialect.MAV_SEVERITY_INFO):
        """
        Отправляет сообщение STATUSTEXT на наземную станцию управления
        """
        if self.mavlink_connection is None:
            return False
        
        try:
            message = dialect.MAVLink_statustext_message(
                severity=severity,
                text=message_text.encode("utf-8")
            )
            self.mavlink_connection.mav.send(message)
            self.get_logger().info(f'MAVLink status sent: {message_text}')
            return True
        except Exception as e:
            self.get_logger().error(f"Error sending MAVLink status: {e}")
            return False
    
    def publish_mavlink_status(self):
        """Публикует статус MAVLink соединения"""
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        msg.frame_id = "connected" if self.mavlink_connection else "disconnected"
        self.mavlink_status_pub.publish(msg)
    
    def init_log_files(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_dir = "tracking_logs"
        os.makedirs(log_dir, exist_ok=True)
        
        self.txt_log_path = os.path.join(log_dir, f"tracking_log_{timestamp}.txt")
        with open(self.txt_log_path, 'w') as f:
            f.write("Time, X Coordinate, Y Coordinate, MAVLinkSent\n")
        
        self.csv_log_path = os.path.join(log_dir, f"tracking_log_{timestamp}.csv")
        with open(self.csv_log_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["Timestamp", "X", "Y", "MAVLinkSent"])
    
    def log_position(self, position, mavlink_sent):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        x, y = position
        
        with open(self.txt_log_path, 'a') as f:
            f.write(f"{timestamp}, {x:.2f}, {y:.2f}, {mavlink_sent}\n")
        
        with open(self.csv_log_path, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([timestamp, f"{x:.2f}", f"{y:.2f}", mavlink_sent])
    
    def image_callback(self, msg):
        """Process incoming image messages"""
        try:
            # Convert ROS Image message to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {str(e)}")
            return
        
        self.frame_counter += 1
        
        # Skip frames based on interval
        if self.frame_counter % self.frame_interval != 0:
            return
        
        # Process frame
        position = self.process_frame(frame)
        
        # Publish position
        if position is not None:
            # Send via MAVLink
            mavlink_sent = False
            if self.mavlink_enabled and self.mavlink_connection is not None:
                mavlink_sent = self.send_mavlink_position(position)
            
            # Publish ROS 2 position
            self.publish_position(position)
            
            # Log position
            self.log_position(position, mavlink_sent)
            
            # Create and publish visualization
            vis_image = self.get_visualization(frame, mavlink_sent)
            self.publish_visualization(vis_image, msg.header)
    
    def process_frame(self, frame):
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame_kp, frame_des = self.detector_1.detectAndCompute(gray_frame, None)
        
        if frame_des is None or len(frame_kp) < 10:
            self.get_logger().warn('Недостаточно ключевых точек')
            return self.last_position
        
        matches = self.matcher.match(self.pano_des, frame_des)
        if len(matches) < 10:
            self.get_logger().warn('Мало совпадений')
            return self.last_position
        
        matches = sorted(matches, key=lambda x: x.distance)[:30]
        src_pts = np.float32([self.pano_kp[m.queryIdx].pt for m in matches]).reshape(-1,1,2)
        dst_pts = np.float32([frame_kp[m.trainIdx].pt for m in matches]).reshape(-1,1,2)
        
        H, _ = cv2.findHomography(dst_pts, src_pts, cv2.RANSAC, 5.0)
        if H is None:
            self.get_logger().error('Ошибка гомографии')
            return self.last_position
        
        h, w = frame.shape[:2]
        center = np.array([[w/2, h/2]], dtype=np.float32).reshape(-1,1,2)
        panorama_center = cv2.perspectiveTransform(center, H)[0][0]
        
        self.positions.append(panorama_center)
        self.last_position = panorama_center
        
        self.get_logger().info(f"Позиция: X={panorama_center[0]:.1f}, Y={panorama_center[1]:.1f}")
        
        return panorama_center
    
    def publish_position(self, position):
        """Publish position as PointStamped message"""
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "panorama"
        
        msg.point.x = float(position[0])
        msg.point.y = float(position[1])
        msg.point.z = 0.0
        
        self.position_pub.publish(msg)
    
    def publish_visualization(self, cv_image, original_header):
        """Publish visualization image"""
        try:
            vis_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            vis_msg.header = original_header
            self.visualization_pub.publish(vis_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing visualization: {str(e)}")
    
    def get_visualization(self, frame, mavlink_sent=False, scale=0.3):
        if self.last_position is None:
            vis = self.panorama.copy()
        else:
            vis = self.panorama.copy()
            
            # Draw tracking history
            point_radius = 10
            for i, pos in enumerate(self.positions):
                color = (0, 255, 0) if i == len(self.positions)-1 else (0, 0, 200)
                cv2.circle(vis, (int(pos[0]), int(pos[1])), point_radius, color, -1)
        
        # Add current camera frame overlay
        small_frame = cv2.resize(frame, None, fx=scale, fy=scale)
        h, w = small_frame.shape[:2]
        vis[10:h+10, 10:w+10] = small_frame
        
        # Add position text
        if self.last_position is not None:
            cv2.putText(vis, f"X: {self.last_position[0]:.1f}, Y: {self.last_position[1]:.1f}", 
                       (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        # Add MAVLink status
        if self.mavlink_enabled:
            mavlink_color = (0, 255, 0) if self.mavlink_connection else (0, 0, 255)
            mavlink_status = "MAVLink: Connected" if self.mavlink_connection else "MAVLink: Disconnected"
            cv2.putText(vis, mavlink_status, (20, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, mavlink_color, 2)
            
            if mavlink_sent:
                cv2.putText(vis, "MAVLink: Position Sent", (20, 120),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # Add ROS 2 status
        cv2.putText(vis, "ROS 2: Active", (20, 150),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Add timestamp
        timestamp = datetime.now().strftime("%H:%M:%S")
        cv2.putText(vis, f"Time: {timestamp}", (20, 180),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return vis
    
    def destroy_node(self):
        """Очистка ресурсов при завершении ноды"""
        # Send final status message
        if self.mavlink_enabled and self.mavlink_connection is not None:
            self.send_statustext_to_gcs("Drone tracker shutting down", dialect.MAV_SEVERITY_INFO)
        
        self.get_logger().info(f"Drone tracker stopped. Logs saved to: {self.txt_log_path}")
        self.get_logger().info(f"Total frames processed: {self.frame_counter}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    # Panorama image path - adjust as needed
    panorama_path = "panorama_output.jpg"
    
    # For ROS2 packages, consider using:
    # from ament_index_python.packages import get_package_share_directory
    # package_share_dir = get_package_share_directory('your_package_name')
    # panorama_path = os.path.join(package_share_dir, 'panorama_output.jpg')
    
    try:
        tracker = OptimizedDroneTracker(
            panorama_path=panorama_path,
            frame_interval=10
        )
        
        start_time = time.time()
        rclpy.spin(tracker)
        elapsed_time = time.time() - start_time
        
        tracker.get_logger().info(f"Время выполнения: {elapsed_time:.2f} сек")
        tracker.get_logger().info("Логи сохранены в 'tracking_logs'")
        
    except ValueError as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        if 'tracker' in locals():
            tracker.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
