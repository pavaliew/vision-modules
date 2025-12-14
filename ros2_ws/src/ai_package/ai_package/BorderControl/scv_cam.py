#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import os
from datetime import datetime
import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect

class RoadFieldDetector(Node):
    def __init__(self):
        super().__init__('road_field_detector')
        
        # Параметры ROS 2
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('log_dir', 'segmentation_logs')
        self.declare_parameter('road_detection_cooldown', 10)
        self.declare_parameter('road_area_threshold', 0.2)
        self.declare_parameter('mavlink_enabled', True)
        self.declare_parameter('mavlink_port', '/dev/ttyACM0')
        
        self.camera_id = self.get_parameter('camera_id').value
        self.log_dir = self.get_parameter('log_dir').value
        self.road_detection_cooldown = self.get_parameter('road_detection_cooldown').value
        self.road_area_threshold = self.get_parameter('road_area_threshold').value
        self.mavlink_enabled = self.get_parameter('mavlink_enabled').value
        self.mavlink_port = self.get_parameter('mavlink_port').value
        
        # Цветовые диапазоны в HSV
        self.lower_field = np.array([35, 40, 40])
        self.upper_field = np.array([85, 255, 255])
        self.lower_road = np.array([0, 0, 100])
        self.upper_road = np.array([179, 50, 220])
        self.kernel = np.ones((5,5), np.uint8)
        
        # MAVLink соединение
        self.mavlink_connection = None
        if self.mavlink_enabled:
            self.setup_mavlink_connection()
        
        # ROS 2 компоненты
        self.bridge = CvBridge()
        
        # Публикаторы
        self.segmentation_pub = self.create_publisher(Image, '/border_control/segmentation', 10)
        self.detection_pub = self.create_publisher(String, '/border_control/detection_result', 10)
        self.confidence_pub = self.create_publisher(Float32, '/border_control/confidence', 10)
        self.status_pub = self.create_publisher(String, '/border_control/status', 10)
        
        # Подписчик на изображение с камеры
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Статистика и таймеры
        self.last_road_detection_time = 0
        self.frame_count = 0
        self.start_time = time.time()
        
        # Настройки логов
        os.makedirs(self.log_dir, exist_ok=True)
        self.setup_new_log()
        
        self.video_initialized = False
        self.segmentation_writer = None
        self.clean_writer = None
        
        self.get_logger().info(f'Road Field Detector started with camera ID: {self.camera_id}')
        self.get_logger().info(f'Log directory: {self.log_dir}')
        self.get_logger().info(f'MAVLink enabled: {self.mavlink_enabled}')
    
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

    def send_statustext_to_gcs(self, message_text, severity=dialect.MAV_SEVERITY_WARNING):
        """
        Отправляет сообщение STATUSTEXT на наземную станцию управления
        
        Args:
            message_text (str): Текст сообщения
            severity (int): Уровень важности (MAV_SEVERITY_*)
        """
        if self.mavlink_connection is None:
            return False
        
        try:
            message = dialect.MAVLink_statustext_message(
                severity=severity,
                text=message_text.encode("utf-8")
            )
            self.mavlink_connection.mav.send(message)
            self.get_logger().info(f'MAVLink message sent to GCS: {message_text}')
            return True
        except Exception as e:
            self.get_logger().error(f"Error sending MAVLink message: {e}")
            return False

    def init_video_writers(self, frame_shape, fps=20):
        """Инициализация видео writers для сегментированного и чистого видео"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        segmentation_path = os.path.join(self.log_dir, f"segmentation_{timestamp}.mp4")
        clean_path = os.path.join(self.log_dir, f"clean_{timestamp}.mp4")
        
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        height, width = frame_shape[:2]
        
        self.segmentation_writer = cv2.VideoWriter(
            segmentation_path, 
            fourcc, 
            fps,
            (width, height)
        )
        
        self.clean_writer = cv2.VideoWriter(
            clean_path, 
            fourcc, 
            fps,
            (width, height)
        )
        
        self.video_initialized = True
        self.get_logger().info(f"Segmentation video: {segmentation_path}")
        self.get_logger().info(f"Clean video: {clean_path}")
    
    def setup_new_log(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_path = f"{self.log_dir}/detection_{timestamp}.txt"
        with open(self.log_path, 'w') as f:
            f.write("Time(ms)\tFrame\tClass\tConfidence\tFieldArea\tRoadArea\tRoadDetected\tMAVLinkMessageSent\n")
        self.get_logger().info(f"Log file: {self.log_path}")
    
    def write_log_entry(self, time_ms, frame_num, class_id, confidence, field_area, road_area, road_detected, mavlink_message_sent):
        with open(self.log_path, 'a') as f:
            f.write(f"{time_ms}\t{frame_num}\t{class_id}\t{confidence:.2f}\t{field_area}\t{road_area}\t{road_detected}\t{mavlink_message_sent}\n")
    
    def image_callback(self, msg):
        """Обработчик входящих изображений от камеры"""
        try:
            # Конвертируем ROS Image в OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            self.frame_count += 1
            current_time_ms = int((time.time() - self.start_time) * 1000)
            current_time_sec = time.time()
            
            # Основная обработка
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Создание масок
            mask_field = cv2.inRange(hsv, self.lower_field, self.upper_field)
            mask_road = cv2.inRange(hsv, self.lower_road, self.upper_road)
            
            # Морфологические операции
            mask_field = cv2.morphologyEx(mask_field, cv2.MORPH_CLOSE, self.kernel)
            mask_road = cv2.morphologyEx(mask_road, cv2.MORPH_OPEN, self.kernel)
            
            # Расчет площадей
            field_area = cv2.countNonZero(mask_field)
            road_area = cv2.countNonZero(mask_road)
            total_area = frame.shape[0] * frame.shape[1]
            
            # Определение класса
            road_detected = False
            message_sent = False
            mavlink_message_sent = False
            
            if road_area > self.road_area_threshold * total_area:
                class_id = 1
                confidence = road_area / (road_area + field_area + 1e-5)
                road_detected = True
                
                # Проверка таймера для отправки сообщения
                if current_time_sec - self.last_road_detection_time >= self.road_detection_cooldown:
                    # ROS 2 сообщение
                    status_msg = String()
                    status_msg.data = "ROAD_DETECTED"
                    self.status_pub.publish(status_msg)
                    message_sent = True
                    
                    # MAVLink сообщение
                    if self.mavlink_enabled and self.mavlink_connection is not None:
                        mavlink_message_sent = self.send_statustext_to_gcs("Road Detected", dialect.MAV_SEVERITY_WARNING)
                    
                    self.last_road_detection_time = current_time_sec
                    self.get_logger().warn(f"Road detected! Confidence: {confidence:.2f}")
                    if mavlink_message_sent:
                        self.get_logger().info("MAVLink message sent to GCS")
            else:
                class_id = 0
                confidence = field_area / (field_area + road_area + 1e-5)
                road_detected = False
            
            # Логирование
            self.write_log_entry(current_time_ms, self.frame_count, class_id, confidence, field_area, road_area, road_detected, mavlink_message_sent)
            
            # Создание сегментированного кадра с визуализацией
            segmentation_frame = frame.copy()
            class_text = "ROAD" if class_id == 1 else "FIELD"
            color = (0, 0, 255) if class_id == 1 else (0, 255, 0)
            
            cv2.putText(segmentation_frame, class_text, (20, 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
            
            # Добавляем сообщение о дороге если обнаружена
            if road_detected:
                time_since_last_msg = current_time_sec - self.last_road_detection_time
                cooldown_remaining = max(0, self.road_detection_cooldown - time_since_last_msg)
                
                if message_sent:
                    cv2.putText(segmentation_frame, "ROAD DETECTED - MESSAGE SENT", (20, 200), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    
                    if mavlink_message_sent:
                        cv2.putText(segmentation_frame, "MAVLINK: MESSAGE SENT TO GCS", (20, 230), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                else:
                    cv2.putText(segmentation_frame, f"ROAD DETECTED - COOLDOWN: {cooldown_remaining:.1f}s", (20, 200), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 100, 255), 2)
            
            # Отрисовка контуров
            contours = cv2.findContours(mask_road if class_id == 1 else mask_field, 
                                      cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
            for cnt in contours:
                if cv2.contourArea(cnt) > 1000:
                    cv2.drawContours(segmentation_frame, [cnt], -1, color, 2)
            
            # Добавление временной метки и статистики
            info_text = f"Time: {current_time_ms/1000:.1f}s | Frame: {self.frame_count}"
            cv2.putText(segmentation_frame, info_text, (20, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            stats_text = f"Field: {field_area} | Road: {road_area}"
            cv2.putText(segmentation_frame, stats_text, (20, 130),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Статус и таймер
            time_since_last_msg = current_time_sec - self.last_road_detection_time
            cooldown_status = f"Cooldown: {time_since_last_msg:.1f}/{self.road_detection_cooldown}s"
            cv2.putText(segmentation_frame, cooldown_status, (20, 170),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            # Статус соединений
            if self.mavlink_enabled:
                mavlink_status = "MAVLink: Connected" if self.mavlink_connection else "MAVLink: Disconnected"
                mavlink_color = (0, 255, 0) if self.mavlink_connection else (0, 0, 255)
                cv2.putText(segmentation_frame, mavlink_status, (20, 190),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, mavlink_color, 2)
            
            cv2.putText(segmentation_frame, "ROS 2: Active", (20, 210),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Инициализация видео writers при первом кадре
            if not self.video_initialized:
                self.init_video_writers(frame.shape)
            
            # Запись кадров в видео
            if self.segmentation_writer is not None:
                self.segmentation_writer.write(segmentation_frame)
            
            if self.clean_writer is not None:
                self.clean_writer.write(frame)
            
            # Публикация результатов в ROS 2
            # Сегментированное изображение
            seg_msg = self.bridge.cv2_to_imgmsg(segmentation_frame, "bgr8")
            self.segmentation_pub.publish(seg_msg)
            
            # Результат детекции
            detection_msg = String()
            detection_msg.data = f"{class_text}_DETECTED"
            self.detection_pub.publish(detection_msg)
            
            # Уверенность
            confidence_msg = Float32()
            confidence_msg.data = confidence
            self.confidence_pub.publish(confidence_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing frame: {e}')
    
    def destroy_node(self):
        """Очистка ресурсов при завершении ноды"""
        if self.segmentation_writer is not None:
            self.segmentation_writer.release()
        if self.clean_writer is not None:
            self.clean_writer.release()
        
        # Закрытие MAVLink соединения
        if self.mavlink_connection is not None:
            self.get_logger().info("Closing MAVLink connection")
        
        self.get_logger().info(f'Road Field Detector stopped. Processed {self.frame_count} frames')
        self.get_logger().info(f'Logs saved to: {self.log_path}')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = RoadFieldDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
