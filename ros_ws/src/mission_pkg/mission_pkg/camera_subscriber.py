#!/usr/bin/env python3

# ============================================================
# camera_subscriber.py
#
# 목적:
#   Gazebo iris_comp 모델의 하향 카메라 토픽을 subscribe
#   ROS Image 메시지를 OpenCV 이미지로 변환해서 확인
#
# 현재 이 노드가 하는 일:
#   1. /down_camera/image_raw 토픽 구독
#   2. cv_bridge로 ROS Image -> OpenCV BGR 이미지 변환
#   3. 이미지 크기, encoding, frame count 로그 출력
#   4. 디버그용 overlay 이미지 생성
#   5. /mission/debug_image 토픽으로 디버그 이미지 publish
#   6. 일정 프레임마다 /project/data/raw에 jpg 저장
# ============================================================


import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
#실시간성을 위해 qos_profile_sensor_data 이용
#History-KEEP_LAST
# Depth-5
# Reliability-BEST_EFFORT
# Durability-VOLATILE

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from pathlib import Path


class CameraSubscriber(Node):
    #하향 카메라 image topic을 subscribe하는 노드.
    #입력:/down_camera/image_raw
    #출력:/mission/debug_image

    def __init__(self):
        super().__init__("camera_subscriber")

        # ------------------------------------------------------------
        # 파라미터 선언
        # ------------------------------------------------------------
        # Ex)ros2 run mission_pkg camera_subscriber --ros-args -p image_topic:=/other/topic

        # 구독할 원본 카메라 토픽
        self.declare_parameter("image_topic", "/down_camera/image_raw")

        # 디버그용 이미지를 publish 토픽
        self.declare_parameter("debug_topic", "/mission/debug_image")

        # 디버그용 이미지 저장 폴더
        self.declare_parameter("save_dir", "/project/data/raw")

        #30프레임마다 log 출력
        self.declare_parameter("log_interval", 30)

        #120프레임마다 이미지 저장
        self.declare_parameter("save_interval", 120)

        # 디버그 이미지 토픽 publish 여부
        self.declare_parameter("publish_debug_image", True)

        # ------------------------------------------------------------
        # 파라미터 값 읽기
        # ------------------------------------------------------------
        # declare_parameter로 기본값을 선언한 뒤 실제 값을 읽어온다.
        self.image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )

        self.debug_topic = (
            self.get_parameter("debug_topic").get_parameter_value().string_value
        )

        save_dir_str = (
            self.get_parameter("save_dir").get_parameter_value().string_value
        )

        self.log_interval = (
            self.get_parameter("log_interval").get_parameter_value().integer_value
        )

        self.save_interval = (
            self.get_parameter("save_interval").get_parameter_value().integer_value
        )

        self.publish_debug_image = (
            self.get_parameter("publish_debug_image").get_parameter_value().bool_value
        )

        # ------------------------------------------------------------
        # CvBridge 생성
        # ------------------------------------------------------------
        # ROS Image -> OpenCV image 변환 or OpenCV image -> ROS Image 변환
        self.bridge = CvBridge()

        # ------------------------------------------------------------
        # 프레임 카운터
        # ------------------------------------------------------------
        # 이미지 callback이 호출될 때마다 1씩 증가
        self.frame_count = 0

        # ------------------------------------------------------------
        # 이미지 저장 폴더 생성
        # ------------------------------------------------------------
        self.save_dir = Path(save_dir_str)
        self.save_dir.mkdir(parents=True, exist_ok=True)

        # ------------------------------------------------------------
        # 카메라 이미지 subscriber 생성
        # ------------------------------------------------------------
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        # ------------------------------------------------------------
        # 디버그 이미지 publisher 생성
        # ------------------------------------------------------------
        #/mission/debug_image publish
        # rqt_image_view에서 /mission/debug_image로 확인
        self.debug_pub = self.create_publisher(
            Image,
            self.debug_topic,
            qos_profile_sensor_data,
        )

        # ------------------------------------------------------------
        # 시작 로그 출력
        # ------------------------------------------------------------
        self.get_logger().info("CameraSubscriber node started.")
        self.get_logger().info(f"Subscribed image topic : {self.image_topic}")
        self.get_logger().info(f"Debug image topic      : {self.debug_topic}")
        self.get_logger().info(f"Image save directory   : {self.save_dir}")
        self.get_logger().info(f"Log interval           : {self.log_interval}")
        self.get_logger().info(f"Save interval          : {self.save_interval}")
        self.get_logger().info(f"Publish debug image    : {self.publish_debug_image}")

    def image_callback(self, msg: Image):

        # ------------------------------------------------------------
        # 1. 프레임 카운트 증가
        # ------------------------------------------------------------
        self.frame_count += 1

        # ------------------------------------------------------------
        # 2. ROS Image -> OpenCV image 변환
        # ------------------------------------------------------------
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )

        except Exception as e:
            # 변환 실패 시 에러 로그를 출력하고 이번 프레임은 처리하지 않는다.
            self.get_logger().error(f"Failed to convert ROS Image to OpenCV: {e}")
            return

        # ------------------------------------------------------------
        # 3. 이미지 형태 확인
        # ------------------------------------------------------------
        if len(frame.shape) == 3:
            height, width, channels = frame.shape
        else:
            # 흑백 이미지로 들어오면 channels를 1로 처리
            height, width = frame.shape
            channels = 1

        # ------------------------------------------------------------
        # 4. 디버그 이미지 생성
        # ------------------------------------------------------------
        # debug_frame은 화면 표시/저장용으로 쓰기 위해 원본 이미지를 복사
        debug_frame = frame.copy()

        #--------------------------------------------
        # 5. 프레임 번호 overlay
        # ------------------------------------------------------------
        cv2.putText(
            debug_frame,
            f"frame: {self.frame_count}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        # ------------------------------------------------------------
        # 6. 이미지 크기/encoding overlay
        # ------------------------------------------------------------
        cv2.putText(
            debug_frame,
            f"{width}x{height}, encoding: {msg.encoding}",
            (20, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        # ------------------------------------------------------------
        # 7. 일정 프레임마다 로그 출력
        # ------------------------------------------------------------
        # log_interval마다 log 출력
        if self.log_interval > 0 and self.frame_count % self.log_interval == 0:
            self.get_logger().info(
                f"Frame {self.frame_count}: "
                f"width={width}, height={height}, channels={channels}, "
                f"encoding={msg.encoding}"
            )

        # ------------------------------------------------------------
        # 8. 디버그 이미지 publish
        # ------------------------------------------------------------
        if self.publish_debug_image:
            try:
                # OpenCV BGR 이미지를 ROS Image 메시지로 다시 변환한다.
                debug_msg = self.bridge.cv2_to_imgmsg(
                    debug_frame,
                    encoding="bgr8",
                )

                # 원본 이미지의 header 복사
                # timestamp, frame_id
                debug_msg.header = msg.header

                # /mission/debug_image 토픽으로 publish한다.
                self.debug_pub.publish(debug_msg)

            except Exception as e:
                self.get_logger().warn(f"Failed to publish debug image: {e}")

        # ------------------------------------------------------------
        # 9. 일정 프레임마다 이미지 저장
        # ------------------------------------------------------------
        if self.save_interval > 0 and self.frame_count % self.save_interval == 0:
            save_path = self.save_dir / f"down_camera_{self.frame_count:06d}.jpg"

            # cv2.imwrite()는 이미지를 파일로 저장한다.
            success = cv2.imwrite(str(save_path), debug_frame)

            if success:
                self.get_logger().info(f"Saved debug image: {save_path}")
            else:
                self.get_logger().warn(f"Failed to save image: {save_path}")


def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Shutting down.")

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()