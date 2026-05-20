#!/usr/bin/env python3

# ============================================================
# line_state_node.py
#
# 목적:
#   BEV 분석 노드에서 나온 선 검출 결과를 받아서
#   현재 라인을 따라갈 수 있는 상태인지 판단한다.
#
# 왜 필요한가?
#   line_follow.py / bev_line_analyzer.py는 매 프레임마다
#   "선이 보인다 / 안 보인다", "오차가 얼마다"를 계산한다.
#
#   하지만 실제 제어에서는 매 프레임 결과를 그대로 믿으면 안 된다.
#
#   예:
#     - 한 프레임만 선이 안 보였다고 바로 LINE_LOST 처리하면 불안정하다.
#     - 순간 노이즈 때문에 error_x가 튀면 드론이 흔들릴 수 있다.
#     - 검출이 몇 프레임 연속 안정적으로 들어왔는지 확인해야 한다.
#
# 이 노드는 그 중간 완충 역할을 한다.
#
# 입력 토픽:
#   /mission/bev_line_detected
#       BEV 기준 선 검출 여부.
#
#   /mission/lateral_error_px
#       BEV 이미지 중앙 기준 선 중심의 x축 픽셀 오차.
#
#   /mission/heading_error_rad
#       BEV 기준 선 방향 오차.
#
# 출력 토픽:
#   /mission/line_state
#       현재 라인 추종 상태.
#       예: "FOLLOW_LINE", "LINE_LOST", "SEARCH_LINE"
#
#   /mission/line_quality
#       0.0 ~ 1.0 사이의 라인 신뢰도.
#
#   /mission/lateral_error_filtered
#       필터링된 lateral error.
#
#   /mission/heading_error_filtered
#       필터링된 heading error.
#
#   /mission/control_ready
#       이후 controller가 제어 명령을 내도 되는 상태인지 여부.
#
# 아직 하지 않는 것:
#   - PX4 Offboard 명령 publish
#   - 속도 명령 계산
#   - PID 제어
#
# 즉, 이 노드는 "비전 결과를 제어 입력으로 쓰기 전 안정화/상태판단" 단계다.
# ============================================================


import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import String


class LineStateNode(Node):
    """
    선 검출 결과를 받아서 라인 추종 상태를 판단하는 ROS2 노드.
    """

    def __init__(self):
        # ------------------------------------------------------------
        # ROS2 노드 이름 설정
        # ------------------------------------------------------------
        super().__init__("line_state_node")

        # ------------------------------------------------------------
        # 입력 토픽 파라미터 선언
        # ------------------------------------------------------------
        self.declare_parameter("detected_topic", "/mission/bev_line_detected")
        self.declare_parameter("lateral_error_topic", "/mission/lateral_error_px")
        self.declare_parameter("heading_error_topic", "/mission/heading_error_rad")

        # ------------------------------------------------------------
        # 출력 토픽 파라미터 선언
        # ------------------------------------------------------------
        self.declare_parameter("state_topic", "/mission/line_state")
        self.declare_parameter("quality_topic", "/mission/line_quality")
        self.declare_parameter("lateral_filtered_topic", "/mission/lateral_error_filtered")
        self.declare_parameter("heading_filtered_topic", "/mission/heading_error_filtered")
        self.declare_parameter("control_ready_topic", "/mission/control_ready")

        # ------------------------------------------------------------
        # 상태 판단 파라미터
        # ------------------------------------------------------------
        # 몇 프레임 연속으로 선이 보여야 FOLLOW_LINE으로 인정할지.
        self.declare_parameter("detect_confirm_count", 3)

        # 몇 프레임 연속으로 선이 안 보여야 LINE_LOST로 인정할지.
        self.declare_parameter("lost_confirm_count", 5)

        # error가 너무 크면 선은 보이더라도 제어 준비 상태가 아니라고 판단한다.
        self.declare_parameter("max_lateral_error_px", 160.0)

        # heading error가 너무 크면 선 방향이 급격히 틀어진 상태로 본다.
        self.declare_parameter("max_heading_error_rad", 0.8)

        # ------------------------------------------------------------
        # 필터 파라미터
        # ------------------------------------------------------------
        # EMA = Exponential Moving Average, 지수이동평균.
        #
        # filtered = alpha * new_value + (1 - alpha) * old_filtered
        #
        # alpha가 클수록:
        #   새 값에 빠르게 반응하지만 노이즈도 많이 따라간다.
        #
        # alpha가 작을수록:
        #   부드럽지만 반응이 느리다.
        self.declare_parameter("ema_alpha_lateral", 0.35)
        self.declare_parameter("ema_alpha_heading", 0.35)

        # line_quality가 줄어들고 회복되는 속도.
        self.declare_parameter("quality_up_step", 0.12)
        self.declare_parameter("quality_down_step", 0.20)

        # 몇 Hz로 상태를 publish할지.
        self.declare_parameter("publish_rate_hz", 20.0)

        # 몇 번 publish마다 로그를 찍을지.
        self.declare_parameter("log_interval", 20)

        # ------------------------------------------------------------
        # 파라미터 값 읽기
        # ------------------------------------------------------------
        self.detected_topic = self.get_parameter("detected_topic").value
        self.lateral_error_topic = self.get_parameter("lateral_error_topic").value
        self.heading_error_topic = self.get_parameter("heading_error_topic").value

        self.state_topic = self.get_parameter("state_topic").value
        self.quality_topic = self.get_parameter("quality_topic").value
        self.lateral_filtered_topic = self.get_parameter("lateral_filtered_topic").value
        self.heading_filtered_topic = self.get_parameter("heading_filtered_topic").value
        self.control_ready_topic = self.get_parameter("control_ready_topic").value

        self.detect_confirm_count = int(self.get_parameter("detect_confirm_count").value)
        self.lost_confirm_count = int(self.get_parameter("lost_confirm_count").value)

        self.max_lateral_error_px = float(self.get_parameter("max_lateral_error_px").value)
        self.max_heading_error_rad = float(self.get_parameter("max_heading_error_rad").value)

        self.ema_alpha_lateral = float(self.get_parameter("ema_alpha_lateral").value)
        self.ema_alpha_heading = float(self.get_parameter("ema_alpha_heading").value)

        self.quality_up_step = float(self.get_parameter("quality_up_step").value)
        self.quality_down_step = float(self.get_parameter("quality_down_step").value)

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.log_interval = int(self.get_parameter("log_interval").value)

        # ------------------------------------------------------------
        # 내부 상태 변수
        # ------------------------------------------------------------
        # 가장 최근에 들어온 선 검출 여부.
        self.raw_detected = False

        # 가장 최근 lateral error.
        self.raw_lateral_error_px = 0.0

        # 가장 최근 heading error.
        self.raw_heading_error_rad = 0.0

        # 필터링된 lateral error.
        self.filtered_lateral_error_px = 0.0

        # 필터링된 heading error.
        self.filtered_heading_error_rad = 0.0

        # 필터가 초기화되었는지 여부.
        # 첫 값이 들어오기 전에는 EMA를 적용할 이전 값이 없다.
        self.filter_initialized = False

        # 연속 검출 카운트.
        self.detect_streak = 0

        # 연속 미검출 카운트.
        self.lost_streak = 0

        # 0.0 ~ 1.0 라인 품질.
        self.line_quality = 0.0

        # 현재 상태 문자열.
        self.line_state = "LINE_LOST"

        # controller가 제어 명령을 내도 되는 상태인지 여부.
        self.control_ready = False

        # publish 횟수 카운터.
        self.publish_count = 0

        # ------------------------------------------------------------
        # Subscriber 생성
        # ------------------------------------------------------------
        self.detected_sub = self.create_subscription(
            Bool,
            self.detected_topic,
            self.detected_callback,
            10,
        )

        self.lateral_sub = self.create_subscription(
            Float32,
            self.lateral_error_topic,
            self.lateral_error_callback,
            10,
        )

        self.heading_sub = self.create_subscription(
            Float32,
            self.heading_error_topic,
            self.heading_error_callback,
            10,
        )

        # ------------------------------------------------------------
        # Publisher 생성
        # ------------------------------------------------------------
        self.state_pub = self.create_publisher(
            String,
            self.state_topic,
            10,
        )

        self.quality_pub = self.create_publisher(
            Float32,
            self.quality_topic,
            10,
        )

        self.lateral_filtered_pub = self.create_publisher(
            Float32,
            self.lateral_filtered_topic,
            10,
        )

        self.heading_filtered_pub = self.create_publisher(
            Float32,
            self.heading_filtered_topic,
            10,
        )

        self.control_ready_pub = self.create_publisher(
            Bool,
            self.control_ready_topic,
            10,
        )

        # ------------------------------------------------------------
        # 주기 타이머 생성
        # ------------------------------------------------------------
        # 입력 callback은 토픽이 들어올 때만 호출된다.
        # 하지만 상태 publish는 일정 주기로 계속 내보내는 게 좋다.
        timer_period = 1.0 / self.publish_rate_hz

        self.timer = self.create_timer(
            timer_period,
            self.timer_callback,
        )

        # ------------------------------------------------------------
        # 시작 로그
        # ------------------------------------------------------------
        self.get_logger().info("LineStateNode started.")
        self.get_logger().info(f"Detected input topic      : {self.detected_topic}")
        self.get_logger().info(f"Lateral error input       : {self.lateral_error_topic}")
        self.get_logger().info(f"Heading error input       : {self.heading_error_topic}")
        self.get_logger().info(f"State output topic        : {self.state_topic}")
        self.get_logger().info(f"Quality output topic      : {self.quality_topic}")
        self.get_logger().info(f"Control ready topic       : {self.control_ready_topic}")
        self.get_logger().info(f"Detect confirm count      : {self.detect_confirm_count}")
        self.get_logger().info(f"Lost confirm count        : {self.lost_confirm_count}")
        self.get_logger().info(f"Max lateral error px      : {self.max_lateral_error_px}")
        self.get_logger().info(f"Max heading error rad     : {self.max_heading_error_rad}")
        self.get_logger().info(f"Publish rate Hz           : {self.publish_rate_hz}")

    def detected_callback(self, msg: Bool):
        """
        /mission/bev_line_detected callback.

        line_follow.py나 bev_line_analyzer.py가 선을 찾았는지 알려준다.
        """

        self.raw_detected = bool(msg.data)

        if self.raw_detected:
            self.detect_streak += 1
            self.lost_streak = 0
        else:
            self.lost_streak += 1
            self.detect_streak = 0

    def lateral_error_callback(self, msg: Float32):
        """
        /mission/lateral_error_px callback.

        BEV 이미지 중앙 기준 선 중심의 x축 오차를 받는다.
        """

        self.raw_lateral_error_px = float(msg.data)

        self.filtered_lateral_error_px = self._ema_update(
            old_value=self.filtered_lateral_error_px,
            new_value=self.raw_lateral_error_px,
            alpha=self.ema_alpha_lateral,
        )

    def heading_error_callback(self, msg: Float32):
        """
        /mission/heading_error_rad callback.

        BEV 기준 선 방향 오차를 받는다.
        """

        self.raw_heading_error_rad = float(msg.data)

        self.filtered_heading_error_rad = self._ema_update(
            old_value=self.filtered_heading_error_rad,
            new_value=self.raw_heading_error_rad,
            alpha=self.ema_alpha_heading,
        )

    def _ema_update(self, old_value: float, new_value: float, alpha: float) -> float:
        """
        EMA 필터 업데이트 함수.

        첫 값이 들어왔을 때는 필터 초기화를 위해 new_value를 그대로 사용한다.
        """

        if not self.filter_initialized:
            self.filter_initialized = True
            return new_value

        alpha = max(0.0, min(1.0, alpha))

        return alpha * new_value + (1.0 - alpha) * old_value

    def timer_callback(self):
        """
        일정 주기로 상태를 판단하고 publish하는 함수.
        """

        self.publish_count += 1

        # ------------------------------------------------------------
        # 1. line_quality 업데이트
        # ------------------------------------------------------------
        # 검출되면 quality를 올리고, 검출 실패하면 quality를 내린다.
        if self.raw_detected:
            self.line_quality += self.quality_up_step
        else:
            self.line_quality -= self.quality_down_step

        self.line_quality = self._clamp(
            self.line_quality,
            0.0,
            1.0,
        )

        # ------------------------------------------------------------
        # 2. 상태 판단
        # ------------------------------------------------------------
        self.line_state = self._decide_line_state()

        # ------------------------------------------------------------
        # 3. control_ready 판단
        # ------------------------------------------------------------
        self.control_ready = self._decide_control_ready()

        # ------------------------------------------------------------
        # 4. publish
        # ------------------------------------------------------------
        self._publish_state()

        # ------------------------------------------------------------
        # 5. 로그 출력
        # ------------------------------------------------------------
        if self.log_interval > 0 and self.publish_count % self.log_interval == 0:
            self.get_logger().info(
                f"state={self.line_state}, "
                f"quality={self.line_quality:.2f}, "
                f"detected={self.raw_detected}, "
                f"detect_streak={self.detect_streak}, "
                f"lost_streak={self.lost_streak}, "
                f"lat_raw={self.raw_lateral_error_px:.1f}, "
                f"lat_filt={self.filtered_lateral_error_px:.1f}, "
                f"head_raw={self.raw_heading_error_rad:.3f}, "
                f"head_filt={self.filtered_heading_error_rad:.3f}, "
                f"control_ready={self.control_ready}"
            )

    def _decide_line_state(self) -> str:
        """
        현재 라인 상태를 결정한다.

        상태:
            FOLLOW_LINE:
                선이 안정적으로 검출되고 추종 가능한 상태.

            SEARCH_LINE:
                방금 선을 놓쳤거나 quality가 애매한 상태.
                바로 lost로 보지 않고 탐색 상태로 둔다.

            LINE_LOST:
                선이 여러 프레임 연속 보이지 않는 상태.
        """

        if self.detect_streak >= self.detect_confirm_count:
            return "FOLLOW_LINE"

        if self.lost_streak >= self.lost_confirm_count:
            return "LINE_LOST"

        return "SEARCH_LINE"

    def _decide_control_ready(self) -> bool:
        """
        이후 controller가 제어 명령을 내도 되는지 판단한다.

        조건:
            1. 상태가 FOLLOW_LINE이어야 한다.
            2. line_quality가 어느 정도 이상이어야 한다.
            3. lateral error가 너무 크지 않아야 한다.
            4. heading error가 너무 크지 않아야 한다.
        """

        if self.line_state != "FOLLOW_LINE":
            return False

        if self.line_quality < 0.5:
            return False

        if abs(self.filtered_lateral_error_px) > self.max_lateral_error_px:
            return False

        if abs(self.filtered_heading_error_rad) > self.max_heading_error_rad:
            return False

        if math.isnan(self.filtered_lateral_error_px):
            return False

        if math.isnan(self.filtered_heading_error_rad):
            return False

        return True

    def _publish_state(self):
        """
        현재 상태를 ROS2 토픽으로 publish한다.
        """

        state_msg = String()
        state_msg.data = self.line_state
        self.state_pub.publish(state_msg)

        quality_msg = Float32()
        quality_msg.data = float(self.line_quality)
        self.quality_pub.publish(quality_msg)

        lateral_msg = Float32()
        lateral_msg.data = float(self.filtered_lateral_error_px)
        self.lateral_filtered_pub.publish(lateral_msg)

        heading_msg = Float32()
        heading_msg.data = float(self.filtered_heading_error_rad)
        self.heading_filtered_pub.publish(heading_msg)

        ready_msg = Bool()
        ready_msg.data = bool(self.control_ready)
        self.control_ready_pub.publish(ready_msg)

    @staticmethod
    def _clamp(value: float, min_value: float, max_value: float) -> float:
        """
        value를 min_value와 max_value 사이로 제한한다.
        """

        return max(min_value, min(max_value, value))


def main(args=None):
    rclpy.init(args=args)

    node = LineStateNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Shutting down.")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()