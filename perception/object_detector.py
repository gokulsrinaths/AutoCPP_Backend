#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import os

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Publishers
        self.detections_pub = self.create_publisher(
            Detection2DArray,
            '/yolo/detections',
            10
        )
        
        self.debug_image_pub = self.create_publisher(
            Image,
            '/yolo/debug_image',
            10
        )
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10
        )
        
        # Load YOLO model
        self.model = self.load_model()
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('confidence_threshold', 0.5),
                ('nms_threshold', 0.4),
                ('debug_view', True)
            ]
        )
        
        # Class names for campus environment
        self.classes = [
            'person', 'bicycle', 'car', 'motorcycle', 'bus', 'truck',
            'traffic light', 'stop sign', 'parking meter', 'bench'
        ]
        
    def load_model(self):
        # Load YOLOv5 model
        model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        model.eval()
        if torch.cuda.is_available():
            model.cuda()
        return model
        
    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Run detection
            detections = self.detect_objects(cv_image)
            
            # Publish detections
            self.publish_detections(detections, msg.header)
            
            # Publish debug image if enabled
            if self.get_parameter('debug_view').value:
                self.publish_debug_image(cv_image, detections)
                
        except Exception as e:
            self.get_logger().error(f'Detection error: {str(e)}')
            
    def detect_objects(self, image):
        # Prepare image for YOLO
        results = self.model(image)
        
        # Process results
        detections = []
        for *xyxy, conf, cls in results.xyxy[0]:
            if conf > self.get_parameter('confidence_threshold').value:
                # Convert to integer coordinates
                x1, y1, x2, y2 = map(int, xyxy)
                
                # Create detection object
                detection = {
                    'bbox': [x1, y1, x2, y2],
                    'class': self.classes[int(cls)],
                    'confidence': float(conf)
                }
                detections.append(detection)
                
        return detections
        
    def publish_detections(self, detections, header):
        # Create Detection2DArray message
        detection_array = Detection2DArray()
        detection_array.header = header
        
        for det in detections:
            detection = Detection2D()
            detection.header = header
            
            # Set bbox
            x1, y1, x2, y2 = det['bbox']
            detection.bbox.center.x = (x1 + x2) / 2
            detection.bbox.center.y = (y1 + y2) / 2
            detection.bbox.size_x = x2 - x1
            detection.bbox.size_y = y2 - y1
            
            # Set class and score
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = det['class']
            hypothesis.hypothesis.score = det['confidence']
            detection.results.append(hypothesis)
            
            detection_array.detections.append(detection)
            
        self.detections_pub.publish(detection_array)
        
    def publish_debug_image(self, image, detections):
        debug_image = image.copy()
        
        # Draw detections
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            label = f"{det['class']} {det['confidence']:.2f}"
            
            # Draw bbox
            cv2.rectangle(debug_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Draw label
            cv2.putText(
                debug_image,
                label,
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2
            )
            
        # Publish debug image
        debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
        self.debug_image_pub.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 