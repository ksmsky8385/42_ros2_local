import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import ObjectDetectionArray, ObjectDetection
from sensor_msgs.msg import Image
from ultralytics import YOLO
from cv_bridge import CvBridge

class YoloPublisher(Node):
    def __init__(self):
        super().__init__('yolo_publisher')
        self.bridge = CvBridge()

	    # YOLOv8 모델 로드 (Nano = 가장 가벼움)
        self.model = YOLO('yolov8n.pt')

        # 구독 & 퍼블리시
        self.create_subscription(Image, 'image_raw', self.callback, 10)
        self.pub = self.create_publisher(ObjectDetectionArray, 'image_yolo', 10)

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(frame, verbose=False)[0]

        array_msg = ObjectDetectionArray()
        array_msg.header = msg.header

        for box in results.boxes:
            detection = ObjectDetection()
            
            detection.class_name = results.names[int(box.cls)]
            detection.confidence = float(box.conf)
            detection.bbox = [int(v) for v in box.xywh[0].tolist()]
            
            array_msg.detections.append(detection)

        self.pub.publish(array_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

