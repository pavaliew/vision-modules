import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import json
import datetime
import os
from cv_bridge import CvBridge

from .inference import (
    YOLO, log_detected_objects, draw_bounding_boxes, 
    process_video_with_detect, detect_dir_files, parse_args
)

# Import MAVLink communication module (you'll need to create this or use an existing one)
from mavlink_communication import MAVLinkCommunication

class YOLODetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        
        # Declare ROS parameters that match your argparse arguments
        self.declare_parameters(
            namespace='',
            parameters=[
                ('path_to_model_w', 'yolo_s_v11_dropout_05_best.pt'),
                ('show_video', False),
                ('save_video', False),
                ('save_logs', True),
                ('output_video_path', 'output_video_FOD.mp4'),
                ('log_file_path', 'detection_logs.json'),
                ('camera_topic', '/camera/image_raw'),
                ('mav_port', '/dev/ttyACM0'),  # MAVLink serial port
                ('mav_baudrate', 57600),  # MAVLink baudrate
            ]
        )
        
        self.bridge = CvBridge()
        self.model = None
        self.result_json = []
        self.log_file_initialized = False
        
        # Initialize MAVLink communication
        self.mav = None
        self.initialize_mavlink()
        
        # Initialize YOLO model
        self.initialize_model()
        
        # Camera subscriber
        camera_topic = self.get_parameter('camera_topic').value
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )
        
        # Publisher for detection results
        self.detection_pub = self.create_publisher(String, 'detection_results', 10)
        self.alert_pub = self.create_publisher(String, 'detection_alerts', 10)
        
        # Video writer for saving video (if enabled)
        self.video_writer = None
        self.frame_count = 0
        
        self.get_logger().info("YOLO Detector Node with MAVLink initialized")
        self.get_logger().info(f"Subscribed to camera topic: {camera_topic}")
    
    def initialize_mavlink(self):
        """Initialize MAVLink connection"""
        mav_port = self.get_parameter('mav_port').value
        mav_baudrate = self.get_parameter('mav_baudrate').value
        
        try:
            self.mav = MAVLinkCommunication(port=mav_port, baudrate=mav_baudrate)
            self.get_logger().info(f"MAVLink initialized on port: {mav_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize MAVLink: {str(e)}")
            self.get_logger().info("MAVLink disabled, continuing without MAVLink communication")
    
    def send_mavlink_alert(self, class_names):
        """Send detection alert via MAVLink"""
        if self.mav is None:
            return
        
        try:
            detection_count = len(class_names)
            self.mav.send_detection_alert(
                detection_count=detection_count, 
                class_names=class_names
            )
            self.get_logger().info(f"MAVLink alert sent for {detection_count} detections: {class_names}")
        except Exception as e:
            self.get_logger().error(f"Failed to send MAVLink alert: {str(e)}")
    
    def send_mavlink_telemetry(self, frame_shape, detections_count):
        """Send telemetry data via MAVLink"""
        if self.mav is None:
            return
        
        try:
            # Example: Send telemetry about detection status
            telemetry_data = {
                'frame_width': frame_shape[1],
                'frame_height': frame_shape[0],
                'detections_count': detections_count,
                'timestamp': self.get_clock().now().nanoseconds
            }
            self.mav.send_telemetry(telemetry_data)
        except Exception as e:
            self.get_logger().error(f"Failed to send MAVLink telemetry: {str(e)}")
    
    def initialize_model(self):
        """Initialize YOLO model with package-relative path support"""
        path_to_model_w = self.get_parameter('path_to_model_w').value
        if not path_to_model_w:
            self.get_logger().error("Parameter 'path_to_model_w' must be set!")
            return
    
        # If it's a relative path, try to find it in package share directory
        if not os.path.isabs(path_to_model_w):
            try:
                from ament_index_python.packages import get_package_share_directory
                package_share_dir = get_package_share_directory('ai_package')
                full_model_path = os.path.join(package_share_dir, 'models', path_to_model_w)
            
                if os.path.exists(full_model_path):
                    path_to_model_w = full_model_path
                    self.get_logger().info(f"Found model in package share: {path_to_model_w}")
                else:
                    self.get_logger().error(f"Model not found in package share: {full_model_path}")
                    return
            except Exception as e:
                self.get_logger().error(f"Error finding package share directory: {e}")
                return
    
        # Now load the model
        try:
            self.model = YOLO(path_to_model_w)
            self.model.fuse()
            self.get_logger().info(f"YOLO model loaded successfully: {path_to_model_w}")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {str(e)}")
    
    def initialize_log_file(self):
        """Initialize the log file with empty array if it doesn't exist"""
        if not self.get_parameter('save_logs').value:
            return
            
        log_file_path = self.get_parameter('log_file_path').value
        
        # Create logs directory if it doesn't exist
        logs_dir = os.path.dirname(log_file_path) if os.path.dirname(log_file_path) else "./logs"
        if not os.path.exists(logs_dir):
            os.makedirs(logs_dir)
            self.get_logger().info(f"Created logs directory: {logs_dir}")
        
        # Check if file exists, if not create it with empty array
        if not os.path.exists(log_file_path):
            try:
                with open(log_file_path, 'w', encoding='utf-8') as json_file:
                    json.dump([], json_file, ensure_ascii=False, indent=4)
                self.get_logger().info(f"Created new log file: {log_file_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to create log file: {str(e)}")
        
        self.log_file_initialized = True
    
    def append_to_log_file(self, detection_data):
        """Append new detection data to the existing log file"""
        if not self.get_parameter('save_logs').value:
            return
            
        log_file_path = self.get_parameter('log_file_path').value
        
        try:
            # Read existing data
            if os.path.exists(log_file_path):
                with open(log_file_path, 'r', encoding='utf-8') as json_file:
                    try:
                        existing_data = json.load(json_file)
                    except json.JSONDecodeError:
                        existing_data = []
            else:
                existing_data = []
            
            # Append new data
            existing_data.append(detection_data)
            
            # Write back to file
            with open(log_file_path, 'w', encoding='utf-8') as json_file:
                json.dump(existing_data, json_file, ensure_ascii=False, indent=4)
            
            self.get_logger().debug(f"Appended detection to log file: {log_file_path}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to append to log file: {str(e)}")
    
    def image_callback(self, msg):
        """Process incoming camera images"""
        if self.model is None:
            return
        
        # Initialize log file on first run if logging is enabled
        if self.get_parameter('save_logs').value and not self.log_file_initialized:
            self.initialize_log_file()
        
        try:
            # Convert ROS Image message to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {str(e)}")
            return
        
        # Initialize video writer if saving is enabled and this is the first frame
        if self.get_parameter('save_video').value and self.video_writer is None:
            self.initialize_video_writer(frame)
        
        # Process frame with YOLO detection (using your existing function)
        results = self.model.predict(frame, iou=0.5, conf=0.6, imgsz=640, verbose=False, half=True)
        
        # Check for detections and publish alerts
        if results and results[0].boxes and len(results[0].boxes) > 0:
            alert_msg = String()
            alert_msg.data = "Внимание, обнаружен посторонний объект! Требуется вмешательство оператора"
            self.alert_pub.publish(alert_msg)
            self.get_logger().warn("Object detected - operator intervention required")
            
            # Send MAVLink alert
            classes = results[0].boxes.cls.cpu().numpy().astype(int)
            class_names = [self.model.names[c] for c in classes]
            self.send_mavlink_alert(class_names)
        
        # Send telemetry data via MAVLink (every frame or periodically)
        if self.frame_count % 10 == 0:  # Send telemetry every 10 frames
            detections_count = len(results[0].boxes) if results and results[0].boxes else 0
            self.send_mavlink_telemetry(frame.shape, detections_count)
        
        # Draw bounding boxes (using your existing function)
        if results[0].boxes is not None:
            frame_with_boxes = draw_bounding_boxes(self.model, frame, results)
            
            # Log detected objects (using your existing function)
            detection_data = {
                "timestamp": self.get_clock().now().nanoseconds,
                "datetime": datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                "detected_objects": log_detected_objects(self.model, results)
            }
            self.result_json.append(detection_data)
            
            # Immediately append to log file if logging is enabled
            if self.get_parameter('save_logs').value:
                self.append_to_log_file(detection_data)
            
            # Publish detection results
            detection_msg = String()
            detection_msg.data = json.dumps(detection_data, ensure_ascii=False)
            self.detection_pub.publish(detection_msg)
        
        # Save frame if video saving is enabled
        if self.video_writer is not None:
            self.video_writer.write(frame)
        
        # Show video if enabled
        if self.get_parameter('show_video').value:
            display_frame = cv2.resize(frame, (0, 0), fx=0.6, fy=0.6)
            cv2.imshow("YOLO Detection", display_frame)
            cv2.waitKey(1)
        
        self.frame_count += 1
        
        # Optional: Periodic backup save (less frequent now since we save on each detection)
        if (self.get_parameter('save_logs').value and 
            self.result_json and 
            self.frame_count % 300 == 0):  # Reduced frequency
            self.save_logs_backup()
    
    def save_logs_backup(self):
        """Periodic backup of all logs (optional safety measure)"""
        if not self.result_json:
            return
            
        try:
            logs_dir = "./logs/backups"
            if not os.path.exists(logs_dir):
                os.makedirs(logs_dir)
            
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            backup_filename = f"detection_backup_{timestamp}.json"
            backup_filepath = os.path.join(logs_dir, backup_filename)
            
            with open(backup_filepath, 'w', encoding='utf-8') as file:
                json.dump(self.result_json, file, ensure_ascii=False, indent=4)
            
            self.get_logger().debug(f"Backup logs saved: {backup_filepath}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to save backup logs: {str(e)}")
    
    def initialize_video_writer(self, frame):
        """Initialize video writer for saving processed video"""
        try:
            output_path = self.get_parameter('output_video_path').value
            day = datetime.date.today()
            time = datetime.datetime.now().strftime("%H-%M")
            filename, ext = os.path.splitext(output_path)
            final_output_path = f"{filename}_{day}_{time}{ext}"
            
            frame_height, frame_width = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video_writer = cv2.VideoWriter(final_output_path, fourcc, 30, (frame_width, frame_height))
            
            self.get_logger().info(f"Video recording started: {final_output_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize video writer: {str(e)}")
    
    def save_logs(self):
        """Save all detection logs to file (for final save or manual trigger)"""
        if not self.get_parameter('save_logs').value or not self.result_json:
            return
            
        try:
            log_file_path = self.get_parameter('log_file_path').value
            
            with open(log_file_path, 'w', encoding='utf-8') as file:
                json.dump(self.result_json, file, ensure_ascii=False, indent=4)
            
            self.get_logger().info(f"All detection logs saved: {log_file_path}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to save logs: {str(e)}")
    
    def destroy_node(self):
        """Cleanup when node is destroyed"""
        # Close MAVLink connection
        if self.mav is not None:
            self.mav.close()
            self.get_logger().info("MAVLink connection closed")
        
        # Release video writer
        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info("Video writer released")
        
        # Final save of logs
        if self.get_parameter('save_logs').value and self.result_json:
            self.save_logs()
        
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = YOLODetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
