#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import json
import datetime
import os
from cv_bridge import CvBridge

# Import all existing functions without modification
import ultralytics
from .communication import MAVLinkCommunication

class LandingTakeoffSafetyNode(Node):
    def __init__(self):
        super().__init__('landing_takeoff_safety_node')
        
        # Declare ROS parameters that match argparse arguments
        self.declare_parameters(
            namespace='',
            parameters=[
                ('weights_path', '/home/jetson/ros2_ws/src/ai_package/ai_package/landing-takeoff-safety/weights/yolov10s.pt'),
                ('roi', '0,0,640,480'),
                ('output_path', 'detection_results.json'),
                ('video_output_path', 'recorded_video.mp4'),
                ('auto_loiter', False),
                ('camera_topic', '/camera/image_raw'),
            ]
        )
        
        self.bridge = CvBridge()
        self.model = None
        self.results_list = []
        self.video_writer = None
        self.frame_count = 0
        self.loiter_activated = False
        self.log_file_initialized = False
        
        # Initialize YOLO model
        self.initialize_model()
        
        Initialize MAVLink communication (optional)
        self.mavlink_comm = MAVLinkCommunication(port='/dev/ttyACM0')
        
        # Camera subscriber
        camera_topic = self.get_parameter('camera_topic').value
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )
        
        # Publishers
        #self.alert_pub = self.create_publisher(String, 'safety_alerts', 10)
        #self.detection_pub = self.create_publisher(String, 'detection_results', 10)
        
        self.get_logger().info("Landing/Takeoff Safety Node initialized")
        self.get_logger().info(f"Subscribed to camera topic: {camera_topic}")
    
    def initialize_model(self):
        """Initialize YOLO model using existing function"""
        weights_path = self.get_parameter('weights_path').value
        if not weights_path:
            self.get_logger().error("Parameter 'weights_path' must be set!")
            return
        
        try:
            # Use existing load_yolo_model function
            self.model = self.load_yolo_model(weights_path)
            self.get_logger().info(f"YOLO model loaded successfully: {weights_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {str(e)}")
    
    def load_yolo_model(self, weights_path):
        return ultralytics.YOLO(model=weights_path)
    
    def initialize_log_file(self):
        """Initialize the log file with empty array if it doesn't exist"""
        output_path = self.get_parameter('output_path').value
        
        # Check if file exists, if not create it with empty array
        if not os.path.exists(output_path):
            try:
                with open(output_path, 'w') as json_file:
                    json.dump([], json_file, indent=4, ensure_ascii=False)
                self.get_logger().info(f"Created new log file: {output_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to create log file: {str(e)}")
        
        self.log_file_initialized = True
    
    def append_to_log_file(self, json_data):
        """Append new detection data to the existing log file"""
        output_path = self.get_parameter('output_path').value
        
        try:
            # Read existing data
            if os.path.exists(output_path):
                with open(output_path, 'r') as json_file:
                    try:
                        existing_data = json.load(json_file)
                    except json.JSONDecodeError:
                        existing_data = []
            else:
                existing_data = []
            
            # Append new data
            existing_data.append(json_data)
            
            # Write back to file
            with open(output_path, 'w') as json_file:
                json.dump(existing_data, json_file, indent=4, ensure_ascii=False)
            
            self.get_logger().debug(f"Appended detection to log file: {output_path}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to append to log file: {str(e)}")
    
    def image_callback(self, msg):
        """Process incoming camera images using existing detection logic"""
        if self.model is None:
            return
        
        # Initialize log file on first run
        if not self.log_file_initialized:
            self.initialize_log_file()
        
        try:
            # Convert ROS Image message to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {str(e)}")
            return
        
        # Initialize video writer if first frame
        if self.video_writer is None:
            self.initialize_video_writer(frame)
        
        # Get ROI parameter
        roi_str = self.get_parameter('roi').value
        roi = tuple(map(int, roi_str.split(',')))
        
        # Extract ROI from frame (using existing logic)
        x, y, w, h = roi
        roi_frame = frame[y:y+h, x:x+w]
        
        # Run YOLO detection (using existing logic)
        results = self.model(roi_frame, verbose=False)
        
        # Check for detections (using existing logic)
        if len(results) > 0 and len(results[0].boxes.xyxy) > 0:
            detected_classes = []
            
            # Draw bounding boxes (using existing logic)
            for box, conf, cls in zip(results[0].boxes.xyxy, results[0].boxes.conf, results[0].boxes.cls):
                x1, y1, x2, y2 = map(int, box[:4])
                
                class_name = self.model.names[int(cls)]
                detected_classes.append(class_name)
                
                label = f'{class_name} {conf:.2f}'
                cv2.rectangle(roi_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(roi_frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            #MAVLink communication (existing logic - commented for safety)
            if hasattr(self, 'mavlink_comm') and self.mavlink_comm:
                 self.mavlink_comm.send_detection_alert(
                     detection_count=len(results[0].boxes.xyxy),
                     class_names=list(set(detected_classes))
                 )
                 
                 # Auto loiter (existing logic)
                 auto_loiter = self.get_parameter('auto_loiter').value
                 if auto_loiter and not self.loiter_activated:
                     self.mavlink_comm.change_to_loiter()
                     self.loiter_activated = True
            
            # Create JSON data (using existing format)
            json_data = {
                'time': datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S %Z%z'),
                'detections': results[0].boxes.xyxy.cpu().numpy().tolist(),
                'scenario': "Замечено препятствие в проверяемой области, требуется присутствие оператора!"
            }
            self.results_list.append(json_data)
            
            # Immediately append to log file
            self.append_to_log_file(json_data)
            
            # Publish alert
            alert_msg = String()
            alert_msg.data = "Замечено препятствие в проверяемой области, требуется присутствие оператора!"
            #self.alert_pub.publish(alert_msg)
            
            # Publish detection results
            detection_msg = String()
            detection_msg.data = json.dumps(json_data, ensure_ascii=False)
            #self.detection_pub.publish(detection_msg)
            
            self.get_logger().warn("Object detected - operator intervention required")
        
        # Draw ROI rectangle on frame (existing visualization)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
        
        # Save frame to video (existing logic)
        if self.video_writer is not None:
            self.video_writer.write(frame)
        
        self.frame_count += 1
        
        # Optional: Periodic backup save (less frequent now since we save on each detection)
        if self.frame_count % 300 == 0:  # Reduced frequency since we save on detection
            self.save_logs_backup()
    
    def save_logs_backup(self):
        """Periodic backup of all logs (optional safety measure)"""
        if not self.results_list:
            return
            
        try:
            output_path = self.get_parameter('output_path').value
            backup_path = f"{output_path}.backup"
            
            with open(backup_path, 'w') as json_file:
                json.dump(self.results_list, json_file, indent=4, ensure_ascii=False)
            
            self.get_logger().debug(f"Backup logs saved: {backup_path}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to save backup logs: {str(e)}")
    
    def initialize_video_writer(self, frame):
        """Initialize video writer using existing logic"""
        try:
            video_output_path = self.get_parameter('video_output_path').value
            
            if video_output_path is None:
                video_output_path = f"recorded_video_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4"
            
            frame_height, frame_width = frame.shape[:2]
            fps = 30  # Default FPS
            
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video_writer = cv2.VideoWriter(video_output_path, fourcc, fps, (frame_width, frame_height))
            
            if not self.video_writer.isOpened():
                self.get_logger().warn(f"Could not open video writer for {video_output_path}")
                self.video_writer = None
            else:
                self.get_logger().info(f"Video recording started: {video_output_path}")
                
        except Exception as e:
            self.get_logger().error(f"Failed to initialize video writer: {str(e)}")
            self.video_writer = None
    
    def save_logs(self):
        """Save all detection logs to file (for final save or manual trigger)"""
        if not self.results_list:
            return
            
        try:
            output_path = self.get_parameter('output_path').value
            
            with open(output_path, 'w') as json_file:
                json.dump(self.results_list, json_file, indent=4, ensure_ascii=False)
            
            self.get_logger().info(f"All detection logs saved: {output_path}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to save logs: {str(e)}")
    
    def destroy_node(self):
        """Cleanup when node is destroyed"""
        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info("Video writer released")
        
        # Save final logs (optional - since we're saving continuously)
        if self.results_list:
            self.save_logs()
        
        #Close MAVLink connection
        if hasattr(self, 'mavlink_comm') and self.mavlink_comm:
        	self.mavlink_comm.close()
        
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = LandingTakeoffSafetyNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
