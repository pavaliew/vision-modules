#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
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

from shapely.geometry import Polygon, Point
import xml.etree.ElementTree as ET
import warnings
warnings.filterwarnings("ignore", category=UserWarning)

class OptimizedDroneTracker(Node):
    def __init__(self, panorama_path, frame_interval=5, max_history=100):
        super().__init__('drone_tracker')
        
        # Parameters
        self.declare_parameter('mavlink_enabled', True)
        self.declare_parameter('mavlink_port', '/dev/ttyACM0')
        self.declare_parameter('kml_file_path', '')
        
        self.mavlink_enabled = self.get_parameter('mavlink_enabled').value
        self.mavlink_port = self.get_parameter('mavlink_port').value
        
        # Load panorama image
        self.panorama = cv2.imread(panorama_path)
        if self.panorama is None:
            self.get_logger().error(f"Не удалось загрузить панораму: {panorama_path}")
            raise ValueError(f"Не удалось загрузить панораму: {panorama_path}")
        
        self.frame_interval = max(1, frame_interval)
        self.frame_counter = 0
        self.max_history = max_history
        
        # KML zones
        self.allowed_zones = []  # Разрешенные зоны
        self.restricted_zones = []  # Запрещенные зоны
        self.load_kml_zones()
        
        # MAVLink connection
        self.mavlink_connection = None
        self.boundary_violation_active = False
        if self.mavlink_enabled:
            self.setup_mavlink_connection()
        
        # Feature detectors and matcher
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
        
        # Subscribe to camera topic
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )
        
        self.init_log_files()
        self.get_logger().info("Drone tracker initialized")
        self.get_logger().info(f"Allowed zones: {len(self.allowed_zones)}, Restricted zones: {len(self.restricted_zones)}")
    
    def load_kml_zones(self):
        """Загрузка разрешенных и запрещенных зон из KML файла"""
        kml_path = self.get_parameter('kml_file_path').value
        
        if not kml_path:
            self.get_logger().info("KML file path not specified. Zone checking disabled.")
            return
            
        if not os.path.exists(kml_path):
            self.get_logger().warn(f"KML file not found: {kml_path}")
            return
            
        try:
            tree = ET.parse(kml_path)
            root = tree.getroot()
            ns = {'kml': 'http://www.opengis.net/kml/2.2'}
            
            for placemark in root.findall('.//kml:Placemark', ns):
                name_elem = placemark.find('kml:name', ns)
                zone_name = name_elem.text if name_elem is not None else "Unnamed zone"
                
                polygon_elem = placemark.find('.//kml:Polygon', ns)
                if polygon_elem is not None:
                    coords_elem = polygon_elem.find('.//kml:coordinates', ns)
                    if coords_elem is not None and coords_elem.text:
                        coord_text = coords_elem.text.strip()
                        points = []
                        
                        for coord in coord_text.split():
                            parts = coord.split(',')
                            if len(parts) >= 2:
                                x, y = float(parts[0]), float(parts[1])
                                points.append((x, y))
                        
                        if len(points) >= 3:
                            polygon = Polygon(points)
                            
                            # Определяем тип зоны по имени
                            zone_name_lower = zone_name.lower()
                            if 'forbidden' in zone_name_lower or 'restricted' in zone_name_lower or 'запрещен' in zone_name_lower:
                                self.restricted_zones.append({
                                    'name': zone_name,
                                    'polygon': polygon,
                                    'points': points
                                })
                                self.get_logger().info(f"Loaded RESTRICTED zone: {zone_name}")
                            else:
                                self.allowed_zones.append({
                                    'name': zone_name,
                                    'polygon': polygon,
                                    'points': points
                                })
                                self.get_logger().info(f"Loaded ALLOWED zone: {zone_name}")
            
            self.get_logger().info(f"Loaded {len(self.allowed_zones)} allowed and {len(self.restricted_zones)} restricted zones")
            
        except Exception as e:
            self.get_logger().error(f"Error loading KML: {e}")
    
    def check_position_validity(self, position):
        """Проверяет валидность позиции относительно зон"""
        if not self.allowed_zones and not self.restricted_zones:
            return True, None  # Нет зон - всё разрешено
        
        point = Point(position[0], position[1])
        
        # Проверка на попадание в запрещенную зону
        for zone in self.restricted_zones:
            if zone['polygon'].contains(point):
                return False, f"In restricted zone: {zone['name']}"
        
        # Проверка на попадание в разрешенную зону (если есть разрешенные зоны)
        if self.allowed_zones:
            in_allowed_zone = False
            for zone in self.allowed_zones:
                if zone['polygon'].contains(point):
                    in_allowed_zone = True
                    break
            
            if not in_allowed_zone:
                return False, "Outside all allowed zones"
        
        return True, "In allowed airspace"
    
    def activate_loiter_mode(self):
        """Активация режима LOITER через MAVLink"""
        if self.mavlink_connection is None:
            self.get_logger().error("MAVLink not connected, cannot activate LOITER")
            return False
        
        try:
            # Команда перехода в режим LOITER
            msg = dialect.MAVLink_command_long_message(
                target_system=self.mavlink_connection.target_system,
                target_component=self.mavlink_connection.target_component,
                command=dialect.MAV_CMD_NAV_LOITER_UNLIM,
                confirmation=0,
                param1=0,  # Empty
                param2=0,  # Empty
                param3=0,  # Empty
                param4=0,  # Yaw angle (0 = current)
                param5=0,  # Latitude (0 = current)
                param6=0,  # Longitude (0 = current)
                param7=0   # Altitude (0 = current)
            )
            self.mavlink_connection.mav.send(msg)
            
            # Статусное сообщение
            status_msg = dialect.MAVLink_statustext_message(
                severity=dialect.MAV_SEVERITY_WARNING,
                text="BOUNDARY VIOLATION - ACTIVATING LOITER MODE".encode("utf-8")
            )
            self.mavlink_connection.mav.send(status_msg)
            
            self.get_logger().warn("LOITER mode activated via MAVLink")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error activating LOITER: {e}")
            return False
    
    def setup_mavlink_connection(self):
        """Настраивает соединение MAVLink"""
        try:
            self.mavlink_connection = utility.mavlink_connection(
                device=self.mavlink_port,
                source_system=1,
                source_component=0
            )
            self.mavlink_connection.wait_heartbeat(timeout=5)
            self.get_logger().info(f"MAVLink connected to {self.mavlink_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect MAVLink: {e}")
            self.mavlink_connection = None
    
    def init_log_files(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_dir = "tracking_logs"
        os.makedirs(log_dir, exist_ok=True)
        
        self.csv_log_path = os.path.join(log_dir, f"tracking_log_{timestamp}.csv")
        with open(self.csv_log_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["Timestamp", "X", "Y", "PositionValid", "ZoneStatus", "MAVLinkAction"])
    
    def log_position(self, position, is_valid, zone_status, mavlink_action=""):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        x, y = position
        
        with open(self.csv_log_path, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([timestamp, f"{x:.2f}", f"{y:.2f}", is_valid, zone_status, mavlink_action])
    
    def image_callback(self, msg):
        """Обработка входящих изображений"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {str(e)}")
            return
        
        self.frame_counter += 1
        
        if self.frame_counter % self.frame_interval != 0:
            return
        
        position = self.process_frame(frame)
        
        if position is not None:
            # Проверка позиции
            is_valid, zone_status = self.check_position_validity(position)
            
            # Обработка нарушения границ
            if not is_valid and not self.boundary_violation_active:
                self.get_logger().error(f"BOUNDARY VIOLATION: {zone_status}")
                
                # Активация LOITER через MAVLink
                if self.mavlink_enabled and self.mavlink_connection:
                    loiter_activated = self.activate_loiter_mode()
                    mavlink_action = "LOITER_ACTIVATED" if loiter_activated else "LOITER_FAILED"
                else:
                    mavlink_action = "MAVLINK_DISABLED"
                
                self.boundary_violation_active = True
            elif is_valid and self.boundary_violation_active:
                self.get_logger().info("Returned to allowed airspace")
                self.boundary_violation_active = False
                mavlink_action = ""
            else:
                mavlink_action = ""
            
            # Публикация позиции
            self.publish_position(position)
            
            # Логирование
            self.log_position(position, is_valid, zone_status, mavlink_action)
            
            # Визуализация
            vis_image = self.get_visualization(frame, position, is_valid, zone_status)
            self.publish_visualization(vis_image, msg.header)
    
    def process_frame(self, frame):
        """Обработка кадра и определение позиции"""
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame_kp, frame_des = self.detector_1.detectAndCompute(gray_frame, None)
        
        if frame_des is None or len(frame_kp) < 10:
            return self.last_position
        
        matches = self.matcher.match(self.pano_des, frame_des)
        if len(matches) < 10:
            return self.last_position
        
        matches = sorted(matches, key=lambda x: x.distance)[:30]
        src_pts = np.float32([self.pano_kp[m.queryIdx].pt for m in matches]).reshape(-1,1,2)
        dst_pts = np.float32([frame_kp[m.trainIdx].pt for m in matches]).reshape(-1,1,2)
        
        H, _ = cv2.findHomography(dst_pts, src_pts, cv2.RANSAC, 5.0)
        if H is None:
            return self.last_position
        
        h, w = frame.shape[:2]
        center = np.array([[w/2, h/2]], dtype=np.float32).reshape(-1,1,2)
        panorama_center = cv2.perspectiveTransform(center, H)[0][0]
        
        self.positions.append(panorama_center)
        self.last_position = panorama_center
        
        return panorama_center
    
    def publish_position(self, position):
        """Публикация позиции дрона"""
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "panorama"
        msg.point.x = float(position[0])
        msg.point.y = float(position[1])
        msg.point.z = 0.0
        self.position_pub.publish(msg)
    
    def publish_visualization(self, cv_image, original_header):
        """Публикация визуализации"""
        try:
            vis_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            vis_msg.header = original_header
            self.visualization_pub.publish(vis_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing visualization: {str(e)}")
    
    def get_visualization(self, frame, position, is_valid, zone_status, scale=0.3):
        """Создание визуализации с зонами"""
        vis = self.panorama.copy()
        
        # Отрисовка разрешенных зон (зеленые)
        for zone in self.allowed_zones:
            points = np.array(zone['points'], dtype=np.int32)
            cv2.polylines(vis, [points], isClosed=True, color=(0, 255, 0), thickness=2)
            if points.size > 0:
                centroid = np.mean(points, axis=0).astype(int)
                cv2.putText(vis, f"A: {zone['name']}", (centroid[0], centroid[1]),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Отрисовка запрещенных зон (красные)
        for zone in self.restricted_zones:
            points = np.array(zone['points'], dtype=np.int32)
            cv2.polylines(vis, [points], isClosed=True, color=(0, 0, 255), thickness=3)
            if points.size > 0:
                centroid = np.mean(points, axis=0).astype(int)
                cv2.putText(vis, f"R: {zone['name']}", (centroid[0], centroid[1]),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        # Отрисовка истории позиций
        for i, pos in enumerate(self.positions):
            color = (0, 255, 0) if is_valid else (0, 0, 255)
            cv2.circle(vis, (int(pos[0]), int(pos[1])), 8, color, -1)
        
        # Текущий видеокадр
        small_frame = cv2.resize(frame, None, fx=scale, fy=scale)
        h, w = small_frame.shape[:2]
        vis[10:h+10, 10:w+10] = small_frame
        
        # Информация о позиции
        cv2.putText(vis, f"Position: {position[0]:.0f}, {position[1]:.0f}", 
                   (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Статус зоны
        status_color = (0, 255, 0) if is_valid else (0, 0, 255)
        status_text = "IN ALLOWED ZONE" if is_valid else f"VIOLATION: {zone_status[:30]}"
        cv2.putText(vis, status_text, (20, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        
        # Статус MAVLink
        if self.boundary_violation_active:
            cv2.putText(vis, "LOITER ACTIVE", (20, 100),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        return vis
    
    def destroy_node(self):
        """Завершение работы"""
        if self.boundary_violation_active:
            self.get_logger().warn("Shutting down with active boundary violation")
        self.get_logger().info(f"Processed {self.frame_counter} frames")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    panorama_path = "panorama_output.jpg"
    
    try:
        tracker = OptimizedDroneTracker(
            panorama_path=panorama_path,
            frame_interval=10
        )
        
        rclpy.spin(tracker)
        
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
