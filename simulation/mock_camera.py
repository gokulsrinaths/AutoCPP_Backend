#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np

class MockCamera(Node):
    def __init__(self):
        super().__init__('mock_camera')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Publishers
        self.image_pub = self.create_publisher(
            Image,
            '/camera/image',
            10
        )
        
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/yolo/detections',
            10
        )
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('fps', 30),
                ('image_width', 640),
                ('image_height', 480),
                ('simulate_objects', True)
            ]
        )
        
        # Create timer for camera updates
        self.create_timer(
            1.0/self.get_parameter('fps').value,
            self.publish_camera_data
        )
        
        # Simulated objects
        self.objects = [
            {'class': 'person', 'prob': 0.95},
            {'class': 'car', 'prob': 0.98},
            {'class': 'bicycle', 'prob': 0.90}
        ]
        
    def publish_camera_data(self):
        # Create mock image
        width = self.get_parameter('image_width').value
        height = self.get_parameter('image_height').value
        image = self.create_mock_image(width, height)
        
        # Convert to ROS message
        img_msg = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera'
        
        # Publish image
        self.image_pub.publish(img_msg)
        
        # Simulate and publish detections
        if self.get_parameter('simulate_objects').value:
            detections = self.create_mock_detections(width, height)
            self.detection_pub.publish(detections)
            
            # Draw detections on image
            self.draw_detections(image, detections)
        
    def create_mock_image(self, width, height):
        # Create a simple gradient background
        image = np.zeros((height, width, 3), dtype=np.uint8)
        for i in range(height):
            for j in range(width):
                image[i,j] = [i % 256, j % 256, (i+j) % 256]
        
        return image
        
    def create_mock_detections(self, width, height):
        detection_array = Detection2DArray()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = 'camera'
        
        # Create random detections
        num_detections = np.random.randint(1, 4)
        for _ in range(num_detections):
            detection = Detection2D()
            
            # Random position
            x = np.random.randint(100, width-100)
            y = np.random.randint(100, height-100)
            w = np.random.randint(50, 150)
            h = np.random.randint(50, 150)
            
            detection.bbox.center.x = float(x)
            detection.bbox.center.y = float(y)
            detection.bbox.size_x = float(w)
            detection.bbox.size_y = float(h)
            
            # Random object
            obj = np.random.choice(self.objects)
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = obj['class']
            hypothesis.hypothesis.score = obj['prob']
            detection.results.append(hypothesis)
            
            detection_array.detections.append(detection)
        
        return detection_array
        
    def draw_detections(self, image, detection_array):
        for detection in detection_array.detections:
            x = int(detection.bbox.center.x)
            y = int(detection.bbox.center.y)
            w = int(detection.bbox.size_x)
            h = int(detection.bbox.size_y)
            
            # Draw rectangle
            cv2.rectangle(
                image,
                (x-w//2, y-h//2),
                (x+w//2, y+h//2),
                (0, 255, 0),
                2
            )
            
            # Draw label
            label = f"{detection.results[0].hypothesis.class_id} {detection.results[0].hypothesis.score:.2f}"
            cv2.putText(
                image,
                label,
                (x-w//2, y-h//2-10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2
            )

def main(args=None):
    rclpy.init(args=args)
    node = MockCamera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 