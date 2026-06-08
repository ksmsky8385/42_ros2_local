import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class EdgeProcessor(Node):
    def __init__(self):
        super().__init__('edge_processor')
        self.bridge = CvBridge()

        # 구독 & 퍼블리시
        self.create_subscription(Image, 'image_raw', self.callback, 10)
        self.pub = self.create_publisher(Image, 'image_edge', 10)

    def callback(self, msg):
        # ROS2 → OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # 그레이스케일 → Canny 엣지
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edge = cv2.Canny(gray, 50, 150)

        # 퍼블리시 (mono8 = 흑백 1채널)
        self.pub.publish(self.bridge.cv2_to_imgmsg(edge, 'mono8'))

def main(args=None):
    rclpy.init(args=args)
    node = EdgeProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

