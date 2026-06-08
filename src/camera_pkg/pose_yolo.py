#!/usr/bin/env python3

import os
import sys
import numpy as np
import cv2
from ultralytics import YOLO

# ROS2 관련 라이브러리
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Wayland/X11 호환성 설정
os.environ["QT_QPA_PLATFORM"] = "xcb"

class PoseEstimationNode(Node):
    def __init__(self):
        super().__init__('pose_estimation_node')
        
        # 1. 파라미터 및 상수 설정
        self.CONFIDENCE_THRESHOLD = 0.6
        self.KEYPOINT_THRESHOLD = 0.5
        
        self.SKELETON_CONNECTIONS = [
            (0, 1), (0, 2), (1, 3), (2, 4),
            (5, 6), (5, 7), (7, 9), (6, 8), (8, 10),
            (11, 12), (11, 13), (13, 15), (12, 14), (14, 16),
            (5, 11), (6, 12)
        ]
        
        self.COLORS = {
            'skeleton': (0, 255, 255),
            'keypoint': (0, 0, 255),
            'bbox': (0, 255, 0),
            'text': (0, 255, 0)
        }

        self.bridge = CvBridge()

        # 2. YOLOv8 포즈 모델 로드
        self.get_logger().info("YOLOv8 포즈 추정 모델을 로드 중...")
        self.model = YOLO('yolov8n-pose.pt')

        # 3. ROS2 구독(Subscriber) 및 발행(Publisher) 설정
        # 다른 카메라 노드(img_pub 등)가 보내주는 raw 이미지를 구독합니다.
        self.img_sub = self.create_subscription(
            Image, 
            'image_raw', 
            self.image_callback, 
            10
        )
        
        # 결과 출력용 토픽들
        self.cmd_pub = self.create_publisher(String, 'robot_command', 10)
        self.img_pub = self.create_publisher(Image, 'camera/image_pose', 10)

        self.get_logger().info("포즈 추정 구독 노드가 성공적으로 시작되었습니다. 'image_raw' 대기 중...")

    def image_callback(self, msg):
        start_time = self.get_clock().now()
        
        # ROS2 이미지 메시지를 OpenCV 이미지(BGR)로 변환
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"이미지 변환 실패: {e}")
            return

        # 이미지 전처리 (거울 효과 방지 및 색상 변환)
        frame = cv2.flip(frame, 1)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # 모델 추론 수행
        results = self.model(frame_rgb, verbose=False)[0]
        keypoints = results.keypoints.xy.cpu().numpy()
        kpt_confs = results.keypoints.conf.cpu().numpy() if results.keypoints.conf is not None else None
        boxes = results.boxes

        current_command = "STOP" # 기본 상태: 정지

        # 객체별 시각화 및 제어 로직 분석
        for i, (box, kpts) in enumerate(zip(boxes, keypoints)):
            kp_conf = kpt_confs[i] if kpt_confs is not None else None
            confidence = box.conf[0].item()
            if confidence < self.CONFIDENCE_THRESHOLD:
                continue

            # 바운딩 박스 시각화
            xmin, ymin, xmax, ymax = map(int, box.xyxy[0].tolist())
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), self.COLORS['bbox'], 2)
            
            text = f"Person {confidence * 100:.1f}%"
            cv2.putText(frame, text, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.COLORS['text'], 2)

            # 키포인트 시각화
            for j, kp in enumerate(kpts):
                x, y = kp
                conf = kp_conf[j] if kp_conf is not None else 1.0
                if x > 0 and y > 0 and conf >= self.KEYPOINT_THRESHOLD:
                    cv2.circle(frame, (int(x), int(y)), 5, self.COLORS['keypoint'], -1)

            # 스켈레톤 라인 시각화
            for connection in self.SKELETON_CONNECTIONS:
                start_idx, end_idx = connection
                s_conf = kp_conf[start_idx] if kp_conf is not None else 1.0
                e_conf = kp_conf[end_idx] if kp_conf is not None else 1.0
                if (len(kpts) > start_idx and len(kpts) > end_idx and
                        kpts[start_idx][0] > 0 and kpts[start_idx][1] > 0 and
                        kpts[end_idx][0] > 0 and kpts[end_idx][1] > 0 and
                        s_conf >= self.KEYPOINT_THRESHOLD and e_conf >= self.KEYPOINT_THRESHOLD):
                    start_x, start_y = int(kpts[start_idx][0]), int(kpts[start_idx][1])
                    end_x, end_y = int(kpts[end_idx][0]), int(kpts[end_idx][1])
                    cv2.line(frame, (start_x, start_y), (end_x, end_y), self.COLORS['skeleton'], 2)

            # [터틀봇 연동 제어 로직]
            # 5: 왼쪽 어깨, 6: 오른쪽 어깨, 9: 왼쪽 손목, 10: 오른쪽 손목
            if len(kpts) > 10 and kp_conf is not None:
                if (kp_conf[5] >= self.KEYPOINT_THRESHOLD and kp_conf[6] >= self.KEYPOINT_THRESHOLD and
                    kp_conf[9] >= self.KEYPOINT_THRESHOLD and kp_conf[10] >= self.KEYPOINT_THRESHOLD):
                    
                    # y축 좌표 비교 (화면 상단이 0에 가깝고 아래로 갈수록 커짐)
                    if kpts[9][1] < kpts[5][1] and kpts[10][1] < kpts[6][1]:
                        current_command = "FORWARD"
                    elif kpts[9][1] < kpts[5][1]:
                        current_command = "LEFT"
                    elif kpts[10][1] < kpts[6][1]:
                        current_command = "RIGHT"

        # 제어 명령 토픽 발행
        cmd_msg = String()
        cmd_msg.data = current_command
        self.cmd_pub.publish(cmd_msg)

        # 프레임 내 상단에 현재 명령 상태 및 FPS 표기
        cv2.putText(frame, f"CMD: {current_command}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

        end_time = self.get_clock().now()
        processing_time = (end_time - start_time).nanoseconds / 1e9
        fps_text = f"FPS: {1.0 / processing_time:.2f}" if processing_time > 0 else "FPS: --"
        cv2.putText(frame, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.COLORS['text'], 2)

        # 결과 시각화 이미지를 다시 ROS2 토픽으로 발행
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.img_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"이미지 발행 실패: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("노드가 사용자에 의해 종료되었습니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()