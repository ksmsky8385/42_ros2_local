import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO
from cv_bridge import CvBridge

class YoloProcessor(Node):
    def __init__(self):
        super().__init__('yolo_processor')
        self.bridge = CvBridge()

        # YOLOv8 모델 로드 (Nano = 가장 가벼움)
        self.model = YOLO('yolov8n.pt')

        # 구독 & 퍼블리시
        self.create_subscription(Image, 'image_raw', self.callback, 10)
        self.pub = self.create_publisher(Image, 'image_yolo', 10)

    def callback(self, msg):
        # ROS2 → OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # YOLOv8 추론
        results = self.model(frame, verbose=False)[0]

        # 박스 그리기
        annotated = results.plot()

        # 퍼블리시
        self.pub.publish(self.bridge.cv2_to_imgmsg(annotated, 'bgr8'))
        # self.get_logger().info('YOLO 발행 중...')

def main(args=None):
    rclpy.init(args=args)
    node = YoloProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

