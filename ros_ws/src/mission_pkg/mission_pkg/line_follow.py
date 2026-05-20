#!/usr/bin/env python3

# ============================================================
# line_follow.py
#
# 현재 목적:
#   하향 카메라 영상에서 바닥의 선을 검출하고,
#   검출된 선의 중심점(cx, cy)과 이미지 중앙 기준 오차(x, y, angle)를 계산한다.
#
# 현재 이 노드가 하는 일:
#   1. /down_camera/image_raw 토픽 수신
#   2. cv_bridge로 ROS Image -> OpenCV BGR 이미지 변환
#   3. BGR -> HSV 변환
#   4. HSV threshold로 파란색 라인 후보 mask 생성
#   5. morphology로 mask 노이즈 및 끊김 보정
#   6. mask에서 흰색 픽셀 좌표 추출
#   7. 이미지 기준점과의 거리 기반 -log weight 계산
#   8. weighted center 계산
#   9. weighted covariance/PCA로 대표 라인 방향 계산
#   10. 기준점에서 대표선으로 수선의 발을 내려 error_x, error_y 계산
#   11. 대표선 방향으로 angle_error 계산
#   12. /mission/line_mask, /mission/line_debug,
#       /mission/line_error_vector, /mission/line_detected publish
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
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3Stamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from pathlib import Path


class LineFollowNode(Node):
    #하향 카메라 영상에서 선을 검출하는 ROS2 노드.
    #입력:/down_camera/image_raw
    #출력:
        #/mission/line_mask
        #/mission/line_debug
        #/mission/line_error_vector
        #/mission/line_detected

    def __init__(self):
        super().__init__("line_follow_node")

        # ------------------------------------------------------------
        # 파라미터 선언
        # ------------------------------------------------------------
        # Ex)ros2 run mission_pkg line_follow --ros-args -p min_area:=300

        # 구독할 원본 카메라 토픽
        self.declare_parameter("image_topic", "/down_camera/image_raw")

        # mask publish 토픽
        self.declare_parameter("mask_topic", "/mission/line_mask")

        # debug image publish 토픽
        self.declare_parameter("debug_topic", "/mission/line_debug")

        # x축 픽셀 오차 publish 토픽
        self.declare_parameter("error_topic", "/mission/line_error_vector")

        # 선 검출 여부 publish 토픽
        self.declare_parameter("detected_topic", "/mission/line_detected")

        # 디버그 이미지 저장 폴더
        self.declare_parameter("save_dir", "/project/data/overlay")
        
        # 30프레임마다 log 출력
        self.declare_parameter("log_interval", 30)

        # 120프레임마다 이미지 저장
        self.declare_parameter("save_interval", 120)

        # contour 최소 면적
        self.declare_parameter("min_area", 200.0)

        # ------------------------------------------------------------
        # HSV threshold 파라미터
        # HSV line mask 안정화 이후 AI mask generator로 교체 예정
        # ------------------------------------------------------------
        # OpenCV HSV 범위:
        #   H: 0 ~ 179 (색상)
        #   S: 0 ~ 255 (채도)
        #   V: 0 ~ 255 (밝기)
        #
        # 파란색 계열 기본값:
        #   H 대략 90~140
        # 추후 heuristic하게 수정
        self.declare_parameter("h_low", 90)
        self.declare_parameter("s_low", 50)
        self.declare_parameter("v_low", 30)

        self.declare_parameter("h_high", 140)
        self.declare_parameter("s_high", 255)
        self.declare_parameter("v_high", 255)

        # ------------------------------------------------------------
        # morphology 설정
        # ------------------------------------------------------------
        # kernel_size:
        #   3이면 3x3 커널
        #   5이면 5x5 커널
        self.declare_parameter("kernel_size", 3)

        # ------------------------------------------------------------
        # 파라미터 값 읽기
        # ------------------------------------------------------------
        self.image_topic = self.get_parameter("image_topic").value
        self.mask_topic = self.get_parameter("mask_topic").value
        self.debug_topic = self.get_parameter("debug_topic").value
        self.error_topic = self.get_parameter("error_topic").value
        self.detected_topic = self.get_parameter("detected_topic").value
        # self.image_topic    = /down_camera/image_raw
        # self.mask_topic     = /mission/line_mask
        # self.debug_topic    = /mission/line_debug
        # self.error_topic    = /mission/line_error_vector
        # self.detected_topic = /mission/line_detected

        self.save_dir = Path(self.get_parameter("save_dir").value)
        self.save_dir.mkdir(parents=True, exist_ok=True)

        self.log_interval = int(self.get_parameter("log_interval").value)
        self.save_interval = int(self.get_parameter("save_interval").value)
        self.min_area = float(self.get_parameter("min_area").value)

        self.h_low = int(self.get_parameter("h_low").value)
        self.s_low = int(self.get_parameter("s_low").value)
        self.v_low = int(self.get_parameter("v_low").value)

        self.h_high = int(self.get_parameter("h_high").value)
        self.s_high = int(self.get_parameter("s_high").value)
        self.v_high = int(self.get_parameter("v_high").value)

        self.kernel_size = int(self.get_parameter("kernel_size").value)

        # ------------------------------------------------------------
        # CvBridge 생성
        # ------------------------------------------------------------
        self.bridge = CvBridge()

        # ------------------------------------------------------------
        # 프레임 카운터
        # ------------------------------------------------------------
        self.frame_count = 0

        # ------------------------------------------------------------
        # Subscriber 생성
        # ------------------------------------------------------------
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        # ------------------------------------------------------------
        # Publisher 생성
        # ------------------------------------------------------------
        # threshold mask 이미지 publish
        self.mask_pub = self.create_publisher(
            Image,
            self.mask_topic,
            qos_profile_sensor_data,
        )

        # contour, 중심점, error가 그려진 debug image publish
        self.debug_pub = self.create_publisher(
            Image,
            self.debug_topic,
            qos_profile_sensor_data,
        )

        # 이미지 중앙 기준 x,y축 / angle 오차 publish
        self.error_pub = self.create_publisher(
            Vector3Stamped,
            self.error_topic,
            10,
        )

        # 선 검출 성공 여부 publish
        self.detected_pub = self.create_publisher(
            Bool,
            self.detected_topic,
            10,
        )

        # ------------------------------------------------------------
        # 시작 로그
        # ------------------------------------------------------------
        self.get_logger().info("LineFollowNode started.")
        self.get_logger().info(f"Subscribed image topic : {self.image_topic}")
        self.get_logger().info(f"Mask topic             : {self.mask_topic}")
        self.get_logger().info(f"Debug topic            : {self.debug_topic}")
        self.get_logger().info(f"Error topic            : {self.error_topic}")
        self.get_logger().info(f"Detected topic         : {self.detected_topic}")
        self.get_logger().info(
            f"HSV lower              : ({self.h_low}, {self.s_low}, {self.v_low})"
        )
        self.get_logger().info(
            f"HSV upper              : ({self.h_high}, {self.s_high}, {self.v_high})"
        )
        self.get_logger().info(f"Min contour area       : {self.min_area}")
        self.get_logger().info(f"Save directory         : {self.save_dir}")

    def image_callback(self, msg: Image):
        """
        카메라 이미지 callback.

        처리 순서:
            1. ROS Image -> OpenCV BGR image
            2. BGR -> HSV
            3. HSV threshold로 blue line mask 생성
            4. morphology로 mask 정리
            5. mask에서 흰색 픽셀 좌표 추출
            6. 기준점과 각 흰색 픽셀 사이의 거리 계산
            7. -log distance weight 계산
            8. weighted center 계산
            9. weighted covariance/PCA로 대표선 방향 계산
            10. 기준점에서 대표선으로 수선의 발 계산
            11. error_x, error_y, angle_error 계산
            12. mask/debug/error/detected publish
        """

        # ------------------------------------------------------------
        # 1. 프레임 카운트 증가
        # ------------------------------------------------------------
        self.frame_count += 1

        # ------------------------------------------------------------
        # 2. ROS Image를 OpenCV BGR 이미지로 변환
        # ------------------------------------------------------------
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # ------------------------------------------------------------
        # 3. 이미지 크기 확인
        # ------------------------------------------------------------
        height, width = frame.shape[:2]

        # 이미지 중앙 x좌표
        image_center_x = width // 2
        image_center_y = height // 2

        # 디버그 표시용 이미지.
        debug_frame = frame.copy()

        # ------------------------------------------------------------
        # 4. BGR -> HSV 변환
        # ------------------------------------------------------------
        # H: Hue, 색상
        # S: Saturation, 채도
        # V: Value, 밝기
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # ------------------------------------------------------------
        # 5. HSV threshold 범위 생성
        # ------------------------------------------------------------
        lower = np.array(
            [self.h_low, self.s_low, self.v_low],
            dtype=np.uint8,
        )

        upper = np.array(
            [self.h_high, self.s_high, self.v_high],
            dtype=np.uint8,
        )

        # ------------------------------------------------------------
        # 6. threshold 적용
        # ------------------------------------------------------------
        #  조건에 맞는 픽셀 = 255, 흰색
        #  조건에 안 맞는 픽셀 = 0, 검은색
        mask = cv2.inRange(hsv, lower, upper)

        # ------------------------------------------------------------
        # 7. morphology로 mask 정리
        # ------------------------------------------------------------
        # MORPH_OPEN: 미세 흰색 노이즈 제거
        # MORPH_CLOSE: 미세 끊어진 흰색 영역을 메꿈
        if self.kernel_size > 1:
            kernel = np.ones(
                (self.kernel_size, self.kernel_size),
                dtype=np.uint8,
            )

            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_OPEN,
                kernel,
            )

            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_CLOSE,
                kernel,
            )

        # ------------------------------------------------------------
        # 8. mask에서 라인 후보 픽셀 좌표 추출
        # ------------------------------------------------------------
        ys, xs = np.where(mask > 0)

        # ------------------------------------------------------------
        # 9. 기본값 초기화
        # ------------------------------------------------------------
        line_detected = False
        
        error_x = 0.0
        error_y = 0.0
        angle_error = 0.0

        weighted_cx = None
        weighted_cy = None

        foot_x = None
        foot_y = None
        line_likeness = 0.0
        white_pixel_count = int(len(xs))

        # ------------------------------------------------------------
        # 10. 기준점 설정
        # ------------------------------------------------------------
        # 현재 프로토타입은 이미지 중앙점을 기준점으로 사용
        # 추후 하드웨어/카메라 실 장착 각도에 맞춰 lookahead point로 수정 필요
        ref_x = float(image_center_x)
        ref_y = float(image_center_y)

        # ------------------------------------------------------------
        # 11. weighted PCA 수행
        # ------------------------------------------------------------
        # min_area 보다 white pixel이 작을 경우 weighted PCA 수행하지 않음
        if white_pixel_count >= int(self.min_area):
            xs_f = xs.astype(np.float32)
            ys_f = ys.astype(np.float32)

            # --------------------------------------------------------
            # 11-1. 기준점 기준 white pixel 거리 계산
            # --------------------------------------------------------
            dx_ref = xs_f - ref_x
            dy_ref = ys_f - ref_y

            distances = np.sqrt(dx_ref * dx_ref + dy_ref * dy_ref)

            # 기준점부터 코너까지 거리값
            radius = 0.5 * np.sqrt(float(width * width + height * height))

            # 비정상적인 radius값일시 반환
            if radius < 1e-6:
                radius = 1.0

            d_norm = distances / radius

            # --------------------------------------------------------
            # 11-2. -log distance weight 계산
            # --------------------------------------------------------
            # 기준점에서 가까울 수록 가중치 부여
            # 현재 Drone의 방향성이 없기에 거리로만 가중치 부여
            # 방향성이 생길 경우 score 계산으로 수정 필요
            # eps 증가시 곡선 완만 / 감소시 가까운 픽셀 가중 증가
            eps = 0.03
            min_weight = 0.05

            d_norm_clamped = np.clip(d_norm, 0.0, 1.0)
            
            raw_weight = -np.log(d_norm_clamped + eps)
            # max raw weight
            raw_near = -np.log(eps)
            # min raw weight
            raw_far = -np.log(1.0 + eps)

            weights = (raw_weight - raw_far) / (raw_near - raw_far)
            weights = np.clip(weights, 0.0, 1.0)

            #최소 min weight 반영
            weights = min_weight + (1.0 - min_weight) * weights

            weights = weights.astype(np.float32)

            weight_sum = float(np.sum(weights))

            # ----------------------------------------------------
            # 11-3. weighted center 계산
            # ----------------------------------------------------
            weighted_cx = float(np.sum(weights * xs_f) / weight_sum)
            weighted_cy = float(np.sum(weights * ys_f) / weight_sum)

            # ----------------------------------------------------
            # 11-4. weighted covariance 계산
            # ----------------------------------------------------
            # white pixel의 분포 계산
            dx = xs_f - weighted_cx
            dy = ys_f - weighted_cy

            cxx = float(np.sum(weights * dx * dx) / weight_sum)
            cxy = float(np.sum(weights * dx * dy) / weight_sum)
            cyy = float(np.sum(weights * dy * dy) / weight_sum)

            covariance = np.array(
                [
                    [cxx, cxy],
                    [cxy, cyy],
                ],
                dtype = np.float32,
            )

            # ----------------------------------------------------
            # 11-5. PCA로 대표 방향 계산
            # ----------------------------------------------------
            # covariance의 가장 큰 eigenvalue 방향이
            # mask 픽셀 분포가 가장 길게 퍼진 방향
            eigen_values, eigen_vectors = np.linalg.eigh(covariance)

            largest_index= int(np.argmax(eigen_values))
            smallest_index = 1 - largest_index

            largest_value = float(eigen_values[largest_index])
            smallest_value = float(eigen_values[smallest_index])

            direction = eigen_vectors[:, largest_index].astype(np.float32)

            vx = float(direction[0])
            vy = float(direction[1])

            norm = np.sqrt(vx * vx + vy * vy)

            if norm > 1e-6:
                vx /= norm
                vy /= norm

                # 방향 벡터의 부호는 PCA 특성상 임의로 바뀔 수 있음
                # y가 양수인 방향, 즉 이미지 아래쪽 방향을 기준으로 통일
                if vy < 0.0:
                    vx = -vx
                    vy = -vy

                # ------------------------------------------------
                # 11-6. line_likeness 계산
                # ------------------------------------------------
                # 1에 가까울수록 선 형태,
                # 0에 가까울수록 덩어리/교차점/노이즈
                if largest_value > 1e-6:
                    line_likeness = 1.0 - (smallest_value / largest_value)
                    line_likeness = float(np.clip(line_likeness, 0.0, 1.0))
                else:
                    line_likeness = 0.0

                # ------------------------------------------------
                # 11-7. 기준점에서 대표선으로 내린 수선의 발 계산
                # ------------------------------------------------
                # 대표선:점 C = weighted center, 방향 v = (vx, vy)
                # 기준점:P = (ref_x, ref_y)
                # 수선의 발:F = C + dot(P - C, v) * v
                pc_x = ref_x - weighted_cx
                pc_y = ref_y - weighted_cy

                projection = pc_x * vx + pc_y * vy

                foot_x = weighted_cx + projection * vx
                foot_y = weighted_cy + projection * vy

                # ------------------------------------------------
                # 11-8. error 계산
                # ------------------------------------------------
                # error는 기준점 P에서 대표선까지의 최단거리 벡터이다.
                error_x = float(foot_x - ref_x)
                error_y = float(foot_y - ref_y)

                # ------------------------------------------------
                # 11-9. angle error 계산
                # ------------------------------------------------
                # 이미지 세로축을 기준으로 대표선의 기울어짐 정도 계산
                # angle_error = 0이면 이미지 세로축과 거의 평행하다는 의미이다.
                angle_error = float(np.arctan2(vx, vy))

                line_detected = True

                # ------------------------------------------------
                # 11-10. debug overlay
                # ------------------------------------------------
                # 기준점 P 표시
                cv2.circle(
                    debug_frame,
                    (int(ref_x), int(ref_y)),
                    7,
                    (255, 0, 0),
                    -1,
                )

                # weighted center C 표시
                cv2.circle(
                    debug_frame,
                    (int(weighted_cx), int(weighted_cy)),
                    7,
                    (0, 0, 255),
                    -1,
                )

                # 수선의 발 F 표시
                cv2.circle(
                    debug_frame,
                    (int(foot_x), int(foot_y)),
                    7,
                    (255, 0, 255),
                    -1,
                )

                # 기준점 P에서 수선의 발 F까지 error vector 표시
                cv2.line(
                    debug_frame,
                    (int(ref_x), int(ref_y)),
                    (int(foot_x), int(foot_y)),
                    (255, 0, 255),
                    2,
                )

                # PCA 대표선 표시
                line_length = max(width, height)

                x1 = int(weighted_cx - vx * line_length)
                y1 = int(weighted_cy - vy * line_length)
                x2 = int(weighted_cx + vx * line_length)
                y2 = int(weighted_cy + vy * line_length)

                cv2.line(
                    debug_frame,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    3,
                )

        # ------------------------------------------------------------
        # 12. 이미지 중앙 기준선 표시
        # ------------------------------------------------------------
        cv2.line(
            debug_frame,
            (image_center_x, 0),
            (image_center_x, height),
            (0, 255, 255),
            2,
        )

        cv2.line(
            debug_frame,
            (0, image_center_y),
            (width, image_center_y),
            (0, 255, 255),
            1,
        )

        # ------------------------------------------------------------
        # 13. debug text 표시
        # ------------------------------------------------------------
        cv2.putText(
            debug_frame,
            f"frame: {self.frame_count}",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            debug_frame,
            f"detected: {line_detected}",
            (20, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0) if line_detected else (0, 0, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            debug_frame,
            f"white_pixels: {white_pixel_count}",
            (20, 105),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            debug_frame,
            f"error_x: {error_x:.1f}, error_y: {error_y:.1f}",
            (20, 140),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (255, 0, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            debug_frame,
            f"angle: {angle_error:.3f} rad",
            (20, 175),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (255, 0, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            debug_frame,
            f"line_likeness: {line_likeness:.2f}",
            (20, 210),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (255, 255, 0),
            2,
            cv2.LINE_AA,
        )

        if weighted_cx is not None and weighted_cy is not None:
            cv2.putText(
                debug_frame,
                f"weighted center: ({weighted_cx:.1f}, {weighted_cy:.1f})",
                (20, 245),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

        if foot_x is not None and foot_y is not None:
            cv2.putText(
                debug_frame,
                f"foot: ({foot_x:.1f}, {foot_y:.1f})",
                (20, 280),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (255, 0, 255),
                2,
                cv2.LINE_AA,
            )

        # ------------------------------------------------------------
        # 14. mask image publish
        # ------------------------------------------------------------
        try:
            mask_msg = self.bridge.cv2_to_imgmsg(
                mask,
                encoding="mono8",
            )
            mask_msg.header = msg.header
            self.mask_pub.publish(mask_msg)

        except Exception as e:
            self.get_logger().warn(f"Failed to publish mask image: {e}")

        # ------------------------------------------------------------
        # 15. debug image publish
        # ------------------------------------------------------------
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(
                debug_frame,
                encoding="bgr8",
            )
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().warn(f"Failed to publish debug image: {e}")

        # ------------------------------------------------------------
        # 16. error vector publish
        # ------------------------------------------------------------
        error_msg = Vector3Stamped()
        error_msg.header = msg.header
        error_msg.vector.x = float(error_x)
        error_msg.vector.y = float(error_y)
        error_msg.vector.z = float(angle_error)
        self.error_pub.publish(error_msg)

        # ------------------------------------------------------------
        # 17. detected publish
        # ------------------------------------------------------------
        detected_msg = Bool()
        detected_msg.data = bool(line_detected)
        self.detected_pub.publish(detected_msg)

        # ------------------------------------------------------------
        # 18. 로그 출력
        # ------------------------------------------------------------
        if self.log_interval > 0 and self.frame_count % self.log_interval == 0:
            self.get_logger().info(
                f"frame={self.frame_count}, "
                f"detected={line_detected}, "
                f"white_pixels={white_pixel_count}, "
                f"error_x={error_x:.1f}, "
                f"error_y={error_y:.1f}, "
                f"angle={angle_error:.3f}, "
                f"line_likeness={line_likeness:.2f}"
            )

        # ------------------------------------------------------------
        # 19. 이미지 저장
        # ------------------------------------------------------------
        if self.save_interval > 0 and self.frame_count % self.save_interval == 0:
            debug_path = self.save_dir / f"line_debug_{self.frame_count:06d}.jpg"
            mask_path = self.save_dir / f"line_mask_{self.frame_count:06d}.png"

            cv2.imwrite(str(debug_path), debug_frame)
            cv2.imwrite(str(mask_path), mask)

            self.get_logger().info(f"Saved debug image: {debug_path}")
            self.get_logger().info(f"Saved mask image : {mask_path}")       
            
def main(args=None):
    rclpy.init(args=args)

    node = LineFollowNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Shutting down.")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()