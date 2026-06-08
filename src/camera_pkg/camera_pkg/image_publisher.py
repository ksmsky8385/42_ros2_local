import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        # 1. 파라미터 선언 (이름, 기본값)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('topic_name', 'image_raw')
        self.declare_parameter('image_size', [320, 240])

        # 2. 파라미터 값 가져오기
        self.rate  = self.get_parameter('publish_rate').value
        self.topic = self.get_parameter('topic_name').value
        self.size  = self.get_parameter('image_size').value

        self.add_on_set_parameters_callback(self.parameter_callback)

        # 1. 퍼블리셔 생성: 타입은 Image, 토픽명은 'image_raw', 큐 크기는 10
        self.publisher_ = self.create_publisher(Image, self.topic, 10)
        # 2. 타이머 설정: 1.0 / self.rate 초마다 timer_callback 실행
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)
        # 3. 웹캠 연결 (0번 카메라)
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()
    
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'publish_rate':
                self.rate = param.value

                self.timer.cancel()
                self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

                self.get_logger().info(f'publish_rate={self.rate}Hz')
            elif param.name == 'image_size':
                self.size = param.value
                self.get_logger().info(f'해상도 변경: {self.size}')
        return SetParametersResult(successful=True)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            resized = cv2.resize(frame, tuple(self.size))
            img_msg = self.bridge.cv2_to_imgmsg(resized, encoding="bgr8")

            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = "camera_link"

            self.publisher_.publish(img_msg)
            # self.get_logger().info('이미지 발행 중...')

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

