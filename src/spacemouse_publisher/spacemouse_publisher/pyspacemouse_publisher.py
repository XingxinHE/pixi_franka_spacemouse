import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32, Float64MultiArray
import pyspacemouse
import math
import os
from itertools import zip_longest


class SpaceMousePublisher(Node):
    """
    A ROS2 Node that publishes 3D mouse input as Twist messages.

    This class initializes a ROS2 publisher to publish geometry_msgs/Twist messages
    based on the input from a 3D SpaceMouse device. It uses the pyspacemouse library
    to read the device state and publishes the corresponding linear and angular
    velocities at a fixed rate.
    """

    def __init__(self):
        super().__init__("spacemouse_publisher")
        self.get_logger().info("Initializing SpaceMouse publisher...")

        self.declare_parameter("operator_position_front", True)
        self._operator_position_front = (
            self.get_parameter("operator_position_front").get_parameter_value().bool_value
        )
        self.get_logger().info(f"Operator position front: {self._operator_position_front}")

        self.declare_parameter("device_path", "")
        self._device_path = self.get_parameter("device_path").get_parameter_value().string_value

        self.declare_parameter("command_mode", "pose")
        self._command_mode = self.get_parameter("command_mode").get_parameter_value().string_value

        self.declare_parameter("publish_rate_hz", 100.0)
        self._publish_rate_hz = (
            self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        )
        if self._publish_rate_hz <= 0.0:
            self.get_logger().warn("publish_rate_hz must be positive. Falling back to 100.0")
            self._publish_rate_hz = 100.0

        self.declare_parameter("target_pose_topic", "target_pose")
        self._target_pose_topic = (
            self.get_parameter("target_pose_topic").get_parameter_value().string_value
        )

        self.declare_parameter("publish_target_pose", True)
        self._publish_target_pose = (
            self.get_parameter("publish_target_pose").get_parameter_value().bool_value
        )

        self.declare_parameter("current_pose_topic", "current_pose")
        self._current_pose_topic = (
            self.get_parameter("current_pose_topic").get_parameter_value().string_value
        )

        self.declare_parameter(
            "target_twist_topic", "franka_controller/target_cartesian_velocity_percent"
        )
        self._target_twist_topic = (
            self.get_parameter("target_twist_topic").get_parameter_value().string_value
        )

        self.declare_parameter("publish_legacy_twist", False)
        self._publish_legacy_twist = (
            self.get_parameter("publish_legacy_twist").get_parameter_value().bool_value
        )

        self.declare_parameter("linear_scale", 0.06)
        self._linear_scale = (
            self.get_parameter("linear_scale").get_parameter_value().double_value
        )

        self.declare_parameter("angular_scale", 0.7)
        self._angular_scale = (
            self.get_parameter("angular_scale").get_parameter_value().double_value
        )

        self.declare_parameter("deadband", 0.05)
        self._deadband = self.get_parameter("deadband").get_parameter_value().double_value

        self.declare_parameter("max_translation_step", 0.003)
        self._max_translation_step = (
            self.get_parameter("max_translation_step").get_parameter_value().double_value
        )

        self.declare_parameter("max_rotation_step", 0.03)
        self._max_rotation_step = (
            self.get_parameter("max_rotation_step").get_parameter_value().double_value
        )

        self.declare_parameter("stale_pose_timeout_sec", 0.5)
        self._stale_pose_timeout_sec = (
            self.get_parameter("stale_pose_timeout_sec").get_parameter_value().double_value
        )

        self.declare_parameter("target_frame_id", "")
        self._target_frame_id = (
            self.get_parameter("target_frame_id").get_parameter_value().string_value
        )

        self.declare_parameter(
            "gripper_width_topic", "gripper_client/target_gripper_width_percent"
        )
        self._gripper_width_topic = (
            self.get_parameter("gripper_width_topic").get_parameter_value().string_value
        )

        self.declare_parameter("publish_gripper_width", True)
        self._publish_gripper_width = (
            self.get_parameter("publish_gripper_width").get_parameter_value().bool_value
        )

        self.declare_parameter("publish_crisp_gripper_command", False)
        self._publish_crisp_gripper_command = (
            self.get_parameter("publish_crisp_gripper_command").get_parameter_value().bool_value
        )

        self.declare_parameter("publish_streamed_teleop", False)
        self._publish_streamed_teleop = (
            self.get_parameter("publish_streamed_teleop").get_parameter_value().bool_value
        )

        self.declare_parameter("streamed_pose_topic", "phone_pose")
        self._streamed_pose_topic = (
            self.get_parameter("streamed_pose_topic").get_parameter_value().string_value
        )

        self.declare_parameter("streamed_gripper_topic", "phone_gripper")
        self._streamed_gripper_topic = (
            self.get_parameter("streamed_gripper_topic").get_parameter_value().string_value
        )

        self.declare_parameter(
            "crisp_gripper_command_topic", "gripper/gripper_position_controller/commands"
        )
        self._crisp_gripper_command_topic = (
            self.get_parameter("crisp_gripper_command_topic").get_parameter_value().string_value
        )

        self._twist_publisher = self.create_publisher(
            Twist, self._target_twist_topic, 10
        )
        self._target_pose_publisher = None
        if self._publish_target_pose:
            self._target_pose_publisher = self.create_publisher(
                PoseStamped, self._target_pose_topic, 10
            )
        self._gripper_width_publisher = None
        if self._publish_gripper_width:
            self._gripper_width_publisher = self.create_publisher(
                Float32, self._gripper_width_topic, 10
            )
        self._crisp_gripper_command_publisher = None
        if self._publish_crisp_gripper_command:
            self._crisp_gripper_command_publisher = self.create_publisher(
                Float64MultiArray, self._crisp_gripper_command_topic, 10
            )

        self._streamed_pose_publisher = None
        self._streamed_gripper_publisher = None
        if self._publish_streamed_teleop:
            self._streamed_pose_publisher = self.create_publisher(
                PoseStamped, self._streamed_pose_topic, 10
            )
            self._streamed_gripper_publisher = self.create_publisher(
                Float32, self._streamed_gripper_topic, 10
            )

        self._latest_pose_msg = None
        self._target_pose_msg = None
        self._latest_pose_time = None
        self._spacemouse_device = None
        self._last_buttons = []
        self.create_subscription(PoseStamped, self._current_pose_topic, self._pose_callback, 10)

        self._timer = self.create_timer(1.0 / self._publish_rate_hz, self._timer_callback)
        self._device_open_success = self._open_spacemouse()

        self.get_logger().info(
            "command_mode=%s, target_pose_topic=%s, current_pose_topic=%s, "
            "publish_legacy_twist=%s"
            % (
                self._command_mode,
                self._target_pose_topic,
                self._current_pose_topic,
                self._publish_legacy_twist,
            )
        )

    def _open_spacemouse(self) -> bool:
        try:
            if self._device_path and hasattr(pyspacemouse, "open_by_path"):
                self._spacemouse_device = pyspacemouse.open_by_path(self._device_path)
                return self._spacemouse_device is not None

            if self._device_path:
                try:
                    result = pyspacemouse.open(path=self._device_path)
                except TypeError:
                    result = pyspacemouse.open()
            else:
                result = pyspacemouse.open()

            if hasattr(result, "read"):
                self._spacemouse_device = result
                return True

            return bool(result)
        except Exception as exc:
            self.get_logger().error(f"Failed to open SpaceMouse: {exc}")
            return False

    def _read_spacemouse(self):
        if self._spacemouse_device is not None:
            return self._spacemouse_device.read()

        if hasattr(pyspacemouse, "read"):
            return pyspacemouse.read()

        return None

    def _handle_button_edges(self, state):
        current_buttons = list(getattr(state, "buttons", []))
        if not current_buttons:
            return

        pressed_buttons = [
            idx
            for idx, (previous, current) in enumerate(
                zip_longest(self._last_buttons, current_buttons, fillvalue=0)
            )
            if current and not previous
        ]

        if pressed_buttons:
            self._button_callback(state, current_buttons, pressed_buttons)

        self._last_buttons = current_buttons

    def _pose_callback(self, msg: PoseStamped):
        self._latest_pose_msg = msg
        self._latest_pose_time = self.get_clock().now()
        if self._target_pose_msg is None:
            self._target_pose_msg = PoseStamped()
            self._target_pose_msg.header = msg.header
            self._target_pose_msg.pose = msg.pose

    def _apply_deadband(self, value: float) -> float:
        return 0.0 if abs(value) < self._deadband else value

    def _clamp(self, value: float, limit: float) -> float:
        if value > limit:
            return limit
        if value < -limit:
            return -limit
        return value

    def _quat_multiply(self, q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return (
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        )

    def _quat_normalize(self, q):
        x, y, z, w = q
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm < 1e-9:
            return (0.0, 0.0, 0.0, 1.0)
        inv = 1.0 / norm
        return (x * inv, y * inv, z * inv, w * inv)

    def _quat_from_rotvec(self, rx: float, ry: float, rz: float):
        angle = math.sqrt(rx * rx + ry * ry + rz * rz)
        if angle < 1e-12:
            return (0.0, 0.0, 0.0, 1.0)
        half = 0.5 * angle
        s = math.sin(half) / angle
        return (rx * s, ry * s, rz * s, math.cos(half))

    def _timer_callback(self):
        if not self._device_open_success:
            return

        if self._latest_pose_msg is None:
            self.get_logger().warn(
                (
                    f"Waiting for '{self._current_pose_topic}' (PoseStamped). "
                    "No current pose received yet, so target commands are not being "
                    "published. Check ROS_DOMAIN_ID and RMW_IMPLEMENTATION "
                    f"(current: {os.getenv('RMW_IMPLEMENTATION', 'unset')})."
                ),
                throttle_duration_sec=2.0,
            )
            return

        if self._latest_pose_time is not None and self._stale_pose_timeout_sec > 0.0:
            age = (self.get_clock().now() - self._latest_pose_time).nanoseconds / 1e9
            if age > self._stale_pose_timeout_sec:
                self.get_logger().warn(
                    f"Current pose is stale ({age:.3f}s). Skipping command publish.",
                    throttle_duration_sec=1.0,
                )
                return

        state = self._read_spacemouse()
        if state is None:
            return

        self._handle_button_edges(state)
        dt = 1.0 / self._publish_rate_hz

        twist_msg = Twist()
        twist_msg.linear.x = self._apply_deadband(-float(state.y))
        twist_msg.linear.y = self._apply_deadband(float(state.x))
        twist_msg.linear.z = self._apply_deadband(float(state.z))
        twist_msg.angular.x = self._apply_deadband(-float(state.roll))
        twist_msg.angular.y = self._apply_deadband(-float(state.pitch))
        twist_msg.angular.z = self._apply_deadband(-float(state.yaw))

        if not self._operator_position_front:
            twist_msg.linear.x *= -1
            twist_msg.linear.y *= -1
            twist_msg.angular.x *= -1
            twist_msg.angular.y *= -1

        if self._publish_legacy_twist or self._command_mode == "twist":
            self._twist_publisher.publish(twist_msg)

        if self._command_mode != "pose":
            return

        if self._target_pose_msg is None:
            return

        dx = self._clamp(twist_msg.linear.x * self._linear_scale * dt, self._max_translation_step)
        dy = self._clamp(twist_msg.linear.y * self._linear_scale * dt, self._max_translation_step)
        dz = self._clamp(twist_msg.linear.z * self._linear_scale * dt, self._max_translation_step)

        droll = self._clamp(
            twist_msg.angular.x * self._angular_scale * dt, self._max_rotation_step
        )
        dpitch = self._clamp(
            twist_msg.angular.y * self._angular_scale * dt, self._max_rotation_step
        )
        dyaw = self._clamp(
            twist_msg.angular.z * self._angular_scale * dt, self._max_rotation_step
        )

        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        if self._target_frame_id:
            target.header.frame_id = self._target_frame_id
        else:
            target.header.frame_id = self._latest_pose_msg.header.frame_id

        target.pose = self._target_pose_msg.pose
        target.pose.position.x += dx
        target.pose.position.y += dy
        target.pose.position.z += dz

        q_current = (
            target.pose.orientation.x,
            target.pose.orientation.y,
            target.pose.orientation.z,
            target.pose.orientation.w,
        )
        q_delta = self._quat_from_rotvec(droll, dpitch, dyaw)
        q_next = self._quat_normalize(self._quat_multiply(q_delta, q_current))
        target.pose.orientation.x = q_next[0]
        target.pose.orientation.y = q_next[1]
        target.pose.orientation.z = q_next[2]
        target.pose.orientation.w = q_next[3]

        self._target_pose_msg = target
        if self._target_pose_publisher is not None:
            self._target_pose_publisher.publish(target)
        if self._streamed_pose_publisher is not None:
            self._streamed_pose_publisher.publish(target)

    def _button_callback(self, state, buttons, pressed_buttons):
        target_gripper_width_percent_msg = Float32()
        gripper_value = None
        if 0 in pressed_buttons:
            self.get_logger().info("Button 1 pressed")
            gripper_value = 0.0

        elif 1 in pressed_buttons:
            self.get_logger().info("Button 2 pressed")
            gripper_value = 1.0

        if gripper_value is None:
            return

        target_gripper_width_percent_msg.data = gripper_value

        if self._gripper_width_publisher is not None:
            self._gripper_width_publisher.publish(target_gripper_width_percent_msg)

        if self._streamed_gripper_publisher is not None:
            streamed_gripper_msg = Float32()
            streamed_gripper_msg.data = gripper_value
            self._streamed_gripper_publisher.publish(streamed_gripper_msg)

        if self._crisp_gripper_command_publisher is not None:
            crisp_msg = Float64MultiArray()
            crisp_msg.data = [gripper_value]
            self._crisp_gripper_command_publisher.publish(crisp_msg)

    def destroy_node(self):
        if self._spacemouse_device is not None and hasattr(self._spacemouse_device, "close"):
            self._spacemouse_device.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    spacemouse_publisher = SpaceMousePublisher()
    rclpy.spin(spacemouse_publisher)
    spacemouse_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
