#!/usr/bin/env python3

# ============================================================
# bev_line_analyzer.py
#
# 목적:
#   line_follow.py에서 만든 /mission/line_mask를 받아서
#   Bird's Eye View(BEV)로 변환하고,
#   BEV 기준으로 선의 중심선 방향과 위치 오차를 계산한다.
#
# 왜 BEV가 필요한가?
#   카메라 원본 이미지는 원근감이 있다.
#   가까운 선은 크게 보이고, 먼 선은 작게 보인다.
#
#   라인트레이싱 제어에서는 바닥을 위에서 내려다본 것처럼
#   평면적으로 보는 것이 더 편하다.
#
#   그래서 perspective transform을 사용해서
#   카메라 이미지를 "위에서 내려다본 이미지"처럼 바꾼다.
#
# 입력 토픽:
#   /mission/line_mask
#       line_follow.py가 publish한 threshold mask.
#       흰색 = 선 후보
#       검은색 = 배경
#
# 출력 토픽:
#   /mission/bev_mask
#       BEV 변환된 mask 이미지.
#
#   /mission/bev_debug
#       BEV 이미지 위에 중심점, 중심선, 오차를 그린 디버그 이미지.
#
#   /mission/lateral_error_px
#       BEV 이미지 중앙 기준 선 중심의 x축 픽셀 오차.
#
#   /mission/heading_error_rad
#       선 방향과 이미지 세로축 사이의 각도 오차.
#
#   /mission/bev_line_detected
#       BEV 기준 선 검출 성공 여부.
#
# 아직 하지 않는 것:
#   - 실제 PX4 제어 명령 전송
#   - 속도 명령 생성
#   - PID 제어
# ============================================================


import math
from pathlib import Path

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import Float32

from cv_bridge import CvBridge


class BevLineAnalyzer(Node):
    """
    /mission/line_mask를 받아 BEV 변환 후
    lateral error와 heading error를 계산하는 노드.
    """

    def __init__(self):
        # ------------------------------------------------------------
        # ROS2 노드 이름 설정
        # ------------------------------------------------------------
        super().__init__("bev_line_analyzer")

        # ------------------------------------------------------------
        # 입력/출력 토픽 파라미터
        # ------------------------------------------------------------
        self.declare_parameter("mask_topic", "/mission/line_mask")
        self.declare_parameter("bev_mask_topic", "/mission/bev_mask")
        self.declare_parameter("bev_debug_topic", "/mission/bev_debug")
        self.declare_parameter("lateral_error_topic", "/mission/lateral_error_px")
        self.declare_parameter("heading_error_topic", "/mission/heading_error_rad")
        self.declare_parameter("detected_topic", "/mission/bev_line_detected")

        # ------------------------------------------------------------
        # 저장/로그 파라미터
        # ------------------------------------------------------------
        self.declare_parameter("save_dir", "/project/data/bev")
        self.declare_parameter("log_interval", 30)
        self.declare_parameter("save_interval", 60)

        # ------------------------------------------------------------
        # BEV 출력 크기
        # ------------------------------------------------------------
        # 출력 BEV 이미지의 크기다.
        # 너무 크면 무겁고, 너무 작으면 정밀도가 떨어진다.
        self.declare_parameter("bev_width", 480)
        self.declare_parameter("bev_height", 640)

        # ------------------------------------------------------------
        # Perspective transform source points
        # ------------------------------------------------------------
        # source point는 원본 mask 이미지에서 BEV로 펼칠 사다리꼴 영역이다.
        #
        # 비율로 설정하는 이유:
        #   카메라 해상도가 바뀌어도 어느 정도 대응하기 위해서다.
        #
        # 좌표 의미:
        #   src_top_left_x_ratio
        #   src_top_y_ratio
        #   src_top_right_x_ratio
        #   src_bottom_left_x_ratio
        #   src_bottom_y_ratio
        #   src_bottom_right_x_ratio
        #
        # 초기값은 대략적인 값이다.
        # 실제 화면을 보면서 반드시 조정해야 한다.
        self.declare_parameter("src_top_left_x_ratio", 0.35)
        self.declare_parameter("src_top_right_x_ratio", 0.65)
        self.declare_parameter("src_top_y_ratio", 0.35)

        self.declare_parameter("src_bottom_left_x_ratio", 0.05)
        self.declare_parameter("src_bottom_right_x_ratio", 0.95)
        self.declare_parameter("src_bottom_y_ratio", 0.95)

        # ------------------------------------------------------------
        # 선 픽셀 검출 관련 파라미터
        # ------------------------------------------------------------
        # BEV mask에서 흰색 픽셀을 선 후보로 본다.
        # 단, 너무 적으면 검출 실패로 처리한다.
        self.declare_parameter("min_white_pixels", 100)

        # 아래쪽/위쪽 관심 영역 비율.
        #
        # bottom band:
        #   드론 가까운 쪽의 선 중심을 보기 위한 영역.
        #
        # top band:
        #   드론 앞쪽 선 방향을 보기 위한 영역.
        #
        # 두 중심점을 연결하면 heading 방향을 대략 계산할 수 있다.
        self.declare_parameter("bottom_band_y_min_ratio", 0.75)
        self.declare_parameter("bottom_band_y_max_ratio", 0.95)

        self.declare_parameter("top_band_y_min_ratio", 0.35)
        self.declare_parameter("top_band_y_max_ratio", 0.55)

        # ------------------------------------------------------------
        # 파라미터 읽기
        # ------------------------------------------------------------
        self.mask_topic = self.get_parameter("mask_topic").value
        self.bev_mask_topic = self.get_parameter("bev_mask_topic").value
        self.bev_debug_topic = self.get_parameter("bev_debug_topic").value
        self.lateral_error_topic = self.get_parameter("lateral_error_topic").value
        self.heading_error_topic = self.get_parameter("heading_error_topic").value
        self.detected_topic = self.get_parameter("detected_topic").value

        self.save_dir = Path(self.get_parameter("save_dir").value)
        self.save_dir.mkdir(parents=True, exist_ok=True)

        self.log_interval = int(self.get_parameter("log_interval").value)
        self.save_interval = int(self.get_parameter("save_interval").value)

        self.bev_width = int(self.get_parameter("bev_width").value)
        self.bev_height = int(self.get_parameter("bev_height").value)

        self.src_top_left_x_ratio = float(
            self.get_parameter("src_top_left_x_ratio").value
        )
        self.src_top_right_x_ratio = float(
            self.get_parameter("src_top_right_x_ratio").value
        )
        self.src_top_y_ratio = float(self.get_parameter("src_top_y_ratio").value)

        self.src_bottom_left_x_ratio = float(
            self.get_parameter("src_bottom_left_x_ratio").value
        )
        self.src_bottom_right_x_ratio = float(
            self.get_parameter("src_bottom_right_x_ratio").value
        )
        self.src_bottom_y_ratio = float(self.get_parameter("src_bottom_y_ratio").value)

        self.min_white_pixels = int(self.get_parameter("min_white_pixels").value)

        self.bottom_band_y_min_ratio = float(
            self.get_parameter("bottom_band_y_min_ratio").value
        )
        self.bottom_band_y_max_ratio = float(
            self.get_parameter("bottom_band_y_max_ratio").value
        )

        self.top_band_y_min_ratio = float(
            self.get_parameter("top_band_y_min_ratio").value
        )
        self.top_band_y_max_ratio = float(
            self.get_parameter("top_band_y_max_ratio").value
        )

        # ------------------------------------------------------------
        # CvBridge 생성
        # ------------------------------------------------------------
        self.bridge = CvBridge()

        # ------------------------------------------------------------
        # 프레임 카운터
        # ------------------------------------------------------------
        self.frame_count = 0

        # ------------------------------------------------------------
        # Subscriber
        # ------------------------------------------------------------
        self.mask_sub = self.create_subscription(
            Image,
            self.mask_topic,
            self.mask_callback,
            qos_profile_sensor_data,
        )

        # ------------------------------------------------------------
        # Publisher
        # ------------------------------------------------------------
        self.bev_mask_pub = self.create_publisher(
            Image,
            self.bev_mask_topic,
            qos_profile_sensor_data,
        )

        self.bev_debug_pub = self.create_publisher(
            Image,
            self.bev_debug_topic,
            qos_profile_sensor_data,
        )

        self.lateral_error_pub = self.create_publisher(
            Float32,
            self.lateral_error_topic,
            10,
        )

        self.heading_error_pub = self.create_publisher(
            Float32,
            self.heading_error_topic,
            10,
        )

        self.detected_pub = self.create_publisher(
            Bool,
            self.detected_topic,
            10,
        )

        # ------------------------------------------------------------
        # 시작 로그
        # ------------------------------------------------------------
        self.get_logger().info("BevLineAnalyzer started.")
        self.get_logger().info(f"Subscribed mask topic     : {self.mask_topic}")
        self.get_logger().info(f"BEV mask topic            : {self.bev_mask_topic}")
        self.get_logger().info(f"BEV debug topic           : {self.bev_debug_topic}")
        self.get_logger().info(f"Lateral error topic       : {self.lateral_error_topic}")
        self.get_logger().info(f"Heading error topic       : {self.heading_error_topic}")
        self.get_logger().info(f"Detected topic            : {self.detected_topic}")
        self.get_logger().info(f"BEV size                  : {self.bev_width}x{self.bev_height}")
        self.get_logger().info(f"Save directory            : {self.save_dir}")

    def mask_callback(self, msg: Image):
        """
        /mission/line_mask가 들어올 때마다 실행되는 callback.
        """

        # ------------------------------------------------------------
        # 1. 프레임 카운트 증가
        # ------------------------------------------------------------
        self.frame_count += 1

        # ------------------------------------------------------------
        # 2. ROS Image -> OpenCV mono mask 변환
        # ------------------------------------------------------------
        try:
            mask = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="mono8",
            )
        except Exception as e:
            self.get_logger().error(f"Failed to convert mask image: {e}")
            return

        # ------------------------------------------------------------
        # 3. 이미지 크기 확인
        # ------------------------------------------------------------
        height, width = mask.shape[:2]

        # ------------------------------------------------------------
        # 4. Perspective transform matrix 계산
        # ------------------------------------------------------------
        # 원본 이미지의 사다리꼴 영역을 BEV 직사각형으로 펼친다.
        src_points = self._make_source_points(width, height)
        dst_points = self._make_destination_points()

        transform_matrix = cv2.getPerspectiveTransform(
            src_points,
            dst_points,
        )

        # ------------------------------------------------------------
        # 5. BEV 변환
        # ------------------------------------------------------------
        bev_mask = cv2.warpPerspective(
            mask,
            transform_matrix,
            (self.bev_width, self.bev_height),
        )

        # ------------------------------------------------------------
        # 6. BEV mask 이진화 보정
        # ------------------------------------------------------------
        # warpPerspective 이후 중간 gray 값이 생길 수 있으므로 다시 threshold한다.
        _, bev_mask = cv2.threshold(
            bev_mask,
            127,
            255,
            cv2.THRESH_BINARY,
        )

        # ------------------------------------------------------------
        # 7. BEV debug image 생성
        # ------------------------------------------------------------
        # mono mask를 BGR로 바꿔서 컬러 overlay를 그릴 수 있게 한다.
        bev_debug = cv2.cvtColor(
            bev_mask,
            cv2.COLOR_GRAY2BGR,
        )

        # ------------------------------------------------------------
        # 8. 선 분석
        # ------------------------------------------------------------
        result = self._analyze_bev_mask(bev_mask)

        line_detected = result["line_detected"]
        bottom_center = result["bottom_center"]
        top_center = result["top_center"]
        lateral_error_px = result["lateral_error_px"]
        heading_error_rad = result["heading_error_rad"]
        white_pixel_count = result["white_pixel_count"]

        # ------------------------------------------------------------
        # 9. debug overlay
        # ------------------------------------------------------------
        self._draw_debug_overlay(
            bev_debug=bev_debug,
            line_detected=line_detected,
            bottom_center=bottom_center,
            top_center=top_center,
            lateral_error_px=lateral_error_px,
            heading_error_rad=heading_error_rad,
            white_pixel_count=white_pixel_count,
        )

        # ------------------------------------------------------------
        # 10. BEV mask publish
        # ------------------------------------------------------------
        try:
            bev_mask_msg = self.bridge.cv2_to_imgmsg(
                bev_mask,
                encoding="mono8",
            )
            bev_mask_msg.header = msg.header
            self.bev_mask_pub.publish(bev_mask_msg)

        except Exception as e:
            self.get_logger().warn(f"Failed to publish BEV mask: {e}")

        # ------------------------------------------------------------
        # 11. BEV debug publish
        # ------------------------------------------------------------
        try:
            bev_debug_msg = self.bridge.cv2_to_imgmsg(
                bev_debug,
                encoding="bgr8",
            )
            bev_debug_msg.header = msg.header
            self.bev_debug_pub.publish(bev_debug_msg)

        except Exception as e:
            self.get_logger().warn(f"Failed to publish BEV debug: {e}")

        # ------------------------------------------------------------
        # 12. lateral error publish
        # ------------------------------------------------------------
        lateral_msg = Float32()
        lateral_msg.data = float(lateral_error_px)
        self.lateral_error_pub.publish(lateral_msg)

        # ------------------------------------------------------------
        # 13. heading error publish
        # ------------------------------------------------------------
        heading_msg = Float32()
        heading_msg.data = float(heading_error_rad)
        self.heading_error_pub.publish(heading_msg)

        # ------------------------------------------------------------
        # 14. detected publish
        # ------------------------------------------------------------
        detected_msg = Bool()
        detected_msg.data = bool(line_detected)
        self.detected_pub.publish(detected_msg)

        # ------------------------------------------------------------
        # 15. 로그 출력
        # ------------------------------------------------------------
        if self.log_interval > 0 and self.frame_count % self.log_interval == 0:
            self.get_logger().info(
                f"frame={self.frame_count}, "
                f"detected={line_detected}, "
                f"white_pixels={white_pixel_count}, "
                f"lateral_error_px={lateral_error_px:.1f}, "
                f"heading_error_rad={heading_error_rad:.3f}"
            )

        # ------------------------------------------------------------
        # 16. 이미지 저장
        # ------------------------------------------------------------
        if self.save_interval > 0 and self.frame_count % self.save_interval == 0:
            mask_path = self.save_dir / f"bev_mask_{self.frame_count:06d}.png"
            debug_path = self.save_dir / f"bev_debug_{self.frame_count:06d}.jpg"

            cv2.imwrite(str(mask_path), bev_mask)
            cv2.imwrite(str(debug_path), bev_debug)

            self.get_logger().info(f"saved BEV mask : {mask_path}")
            self.get_logger().info(f"saved BEV debug: {debug_path}")

    def _make_source_points(self, width: int, height: int) -> np.ndarray:
        """
        원본 이미지에서 BEV로 펼칠 사다리꼴 source points를 만든다.

        반환 순서:
            top-left
            top-right
            bottom-right
            bottom-left
        """

        top_y = height * self.src_top_y_ratio
        bottom_y = height * self.src_bottom_y_ratio

        top_left_x = width * self.src_top_left_x_ratio
        top_right_x = width * self.src_top_right_x_ratio

        bottom_left_x = width * self.src_bottom_left_x_ratio
        bottom_right_x = width * self.src_bottom_right_x_ratio

        return np.float32(
            [
                [top_left_x, top_y],
                [top_right_x, top_y],
                [bottom_right_x, bottom_y],
                [bottom_left_x, bottom_y],
            ]
        )

    def _make_destination_points(self) -> np.ndarray:
        """
        BEV 출력 이미지에서 source points가 대응될 destination points를 만든다.

        출력은 직사각형이다.
        """

        return np.float32(
            [
                [0, 0],
                [self.bev_width - 1, 0],
                [self.bev_width - 1, self.bev_height - 1],
                [0, self.bev_height - 1],
            ]
        )

    def _analyze_bev_mask(self, bev_mask: np.ndarray) -> dict:
        """
        BEV mask에서 선의 위치와 방향을 계산한다.

        계산 방식:
            1. 흰색 픽셀 개수 확인
            2. 아래쪽 band에서 중심 x 계산
            3. 위쪽 band에서 중심 x 계산
            4. 아래쪽 중심 기준 lateral_error 계산
            5. 위/아래 중심점 연결로 heading_error 계산
        """

        white_pixels = np.column_stack(
            np.where(bev_mask > 0)
        )

        white_pixel_count = len(white_pixels)

        if white_pixel_count < self.min_white_pixels:
            return {
                "line_detected": False,
                "bottom_center": None,
                "top_center": None,
                "lateral_error_px": 0.0,
                "heading_error_rad": 0.0,
                "white_pixel_count": white_pixel_count,
            }

        bottom_center = self._compute_band_center(
            bev_mask,
            self.bottom_band_y_min_ratio,
            self.bottom_band_y_max_ratio,
        )

        top_center = self._compute_band_center(
            bev_mask,
            self.top_band_y_min_ratio,
            self.top_band_y_max_ratio,
        )

        if bottom_center is None:
            return {
                "line_detected": False,
                "bottom_center": None,
                "top_center": top_center,
                "lateral_error_px": 0.0,
                "heading_error_rad": 0.0,
                "white_pixel_count": white_pixel_count,
            }

        image_center_x = self.bev_width / 2.0

        lateral_error_px = float(bottom_center[0] - image_center_x)

        heading_error_rad = 0.0

        if top_center is not None:
            dx = float(top_center[0] - bottom_center[0])
            dy = float(bottom_center[1] - top_center[1])

            if abs(dy) > 1e-6:
                heading_error_rad = math.atan2(dx, dy)

        return {
            "line_detected": True,
            "bottom_center": bottom_center,
            "top_center": top_center,
            "lateral_error_px": lateral_error_px,
            "heading_error_rad": heading_error_rad,
            "white_pixel_count": white_pixel_count,
        }

    def _compute_band_center(
        self,
        bev_mask: np.ndarray,
        y_min_ratio: float,
        y_max_ratio: float,
    ):
        """
        BEV mask의 특정 y band에서 흰색 픽셀 중심을 계산한다.

        반환:
            (cx, cy) 또는 None
        """

        y_min = int(self.bev_height * y_min_ratio)
        y_max = int(self.bev_height * y_max_ratio)

        y_min = max(0, min(self.bev_height - 1, y_min))
        y_max = max(0, min(self.bev_height, y_max))

        if y_max <= y_min:
            return None

        band = bev_mask[y_min:y_max, :]

        ys, xs = np.where(band > 0)

        if len(xs) == 0:
            return None

        cx = int(np.mean(xs))
        cy = int(np.mean(ys) + y_min)

        return (cx, cy)

    def _draw_debug_overlay(
        self,
        bev_debug: np.ndarray,
        line_detected: bool,
        bottom_center,
        top_center,
        lateral_error_px: float,
        heading_error_rad: float,
        white_pixel_count: int,
    ):
        """
        BEV debug image에 중심선, 중심점, 오차 정보를 그린다.
        """

        center_x = self.bev_width // 2

        # 이미지 중앙선
        cv2.line(
            bev_debug,
            (center_x, 0),
            (center_x, self.bev_height),
            (0, 255, 255),
            2,
        )

        # bottom band 표시
        bottom_y_min = int(self.bev_height * self.bottom_band_y_min_ratio)
        bottom_y_max = int(self.bev_height * self.bottom_band_y_max_ratio)

        cv2.rectangle(
            bev_debug,
            (0, bottom_y_min),
            (self.bev_width - 1, bottom_y_max),
            (255, 255, 0),
            2,
        )

        # top band 표시
        top_y_min = int(self.bev_height * self.top_band_y_min_ratio)
        top_y_max = int(self.bev_height * self.top_band_y_max_ratio)

        cv2.rectangle(
            bev_debug,
            (0, top_y_min),
            (self.bev_width - 1, top_y_max),
            (255, 0, 255),
            2,
        )

        if bottom_center is not None:
            cv2.circle(
                bev_debug,
                bottom_center,
                7,
                (0, 0, 255),
                -1,
            )

            cv2.line(
                bev_debug,
                (center_x, bottom_center[1]),
                bottom_center,
                (255, 0, 0),
                2,
            )

        if top_center is not None:
            cv2.circle(
                bev_debug,
                top_center,
                7,
                (0, 255, 0),
                -1,
            )

        if bottom_center is not None and top_center is not None:
            cv2.line(
                bev_debug,
                bottom_center,
                top_center,
                (0, 255, 0),
                3,
            )

        cv2.putText(
            bev_debug,
            f"detected: {line_detected}",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0) if line_detected else (0, 0, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            bev_debug,
            f"white_pixels: {white_pixel_count}",
            (20, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            bev_debug,
            f"lateral_error: {lateral_error_px:.1f} px",
            (20, 105),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 0, 0),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            bev_debug,
            f"heading_error: {heading_error_rad:.3f} rad",
            (20, 140),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 0, 255),
            2,
            cv2.LINE_AA,
        )


def main(args=None):
    rclpy.init(args=args)

    node = BevLineAnalyzer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Shutting down.")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()