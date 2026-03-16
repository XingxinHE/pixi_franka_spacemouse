import rclpy
from franka_msgs.action import Grasp
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class CrispPyFrankaHandAdapter(Node):
    """Bridge CRISP-style gripper commands to Franka gripper actions."""

    def __init__(self):
        super().__init__("crisp_py_franka_hand_adapter")

        self.declare_parameter(
            "command_topic", "gripper/gripper_position_controller/commands"
        )
        self.declare_parameter("joint_state_topic", "gripper/joint_states")
        self.declare_parameter("franka_gripper_namespace", "franka_gripper")
        self.declare_parameter("publish_joint_state_hz", 50.0)
        self.declare_parameter("open_threshold", 0.07)
        self.declare_parameter("toggle_threshold", 0.04)
        self.declare_parameter("open_width", 0.08)
        self.declare_parameter("close_width", 0.0)
        self.declare_parameter("gripper_speed", 0.1)
        self.declare_parameter("gripper_force", 50.0)
        self.declare_parameter("epsilon_inner", 0.01)
        self.declare_parameter("epsilon_outer", 0.08)

        self._command_topic = self.get_parameter("command_topic").value
        self._joint_state_topic = self.get_parameter("joint_state_topic").value
        self._franka_ns = self.get_parameter("franka_gripper_namespace").value
        self._publish_joint_state_hz = float(
            self.get_parameter("publish_joint_state_hz").value
        )
        self._open_threshold = float(self.get_parameter("open_threshold").value)
        self._toggle_threshold = float(self.get_parameter("toggle_threshold").value)
        self._open_width = float(self.get_parameter("open_width").value)
        self._close_width = float(self.get_parameter("close_width").value)
        self._gripper_speed = float(self.get_parameter("gripper_speed").value)
        self._gripper_force = float(self.get_parameter("gripper_force").value)
        self._epsilon_inner = float(self.get_parameter("epsilon_inner").value)
        self._epsilon_outer = float(self.get_parameter("epsilon_outer").value)

        self._is_closing = False
        self._current_width = None
        self._cb_group = ReentrantCallbackGroup()

        self._grasp_client = ActionClient(
            self,
            Grasp,
            f"{self._franka_ns}/grasp",
            callback_group=self._cb_group,
        )

        self.create_subscription(
            Float64MultiArray,
            self._command_topic,
            self._command_callback,
            qos_profile=qos_profile_system_default,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            JointState,
            f"{self._franka_ns}/joint_states",
            self._franka_joint_state_callback,
            qos_profile=qos_profile_system_default,
            callback_group=self._cb_group,
        )
        self._joint_state_publisher = self.create_publisher(
            JointState,
            self._joint_state_topic,
            qos_profile=qos_profile_system_default,
            callback_group=self._cb_group,
        )

        if self._publish_joint_state_hz > 0.0:
            self.create_timer(
                1.0 / self._publish_joint_state_hz,
                self._publish_crisp_joint_state,
                callback_group=self._cb_group,
            )

        self.get_logger().info(
            "Adapter started: command_topic=%s franka_ns=%s"
            % (self._command_topic, self._franka_ns)
        )
        self.get_logger().info("Waiting for action server: %s/grasp" % self._franka_ns)

    def _franka_joint_state_callback(self, msg: JointState):
        if len(msg.position) >= 2:
            self._current_width = float(msg.position[0] + msg.position[1])

    def _publish_crisp_joint_state(self):
        if self._current_width is None or self._open_width <= 0.0:
            return

        crisp_joint = JointState()
        crisp_joint.header.stamp = self.get_clock().now().to_msg()
        crisp_joint.name = ["gripper_joint"]
        crisp_joint.position = [self._current_width / self._open_width]
        crisp_joint.effort = [0.0]
        self._joint_state_publisher.publish(crisp_joint)

    def _command_callback(self, msg: Float64MultiArray):
        if not msg.data:
            return

        if not self._grasp_client.server_is_ready():
            if not self._grasp_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().warn(
                    f"{self._franka_ns}/grasp not ready. Ignoring gripper command.",
                    throttle_duration_sec=2.0,
                )
                return

        command = float(msg.data[0])
        is_open = (
            self._current_width is None or self._current_width > self._open_threshold
        )

        if command <= self._toggle_threshold and is_open and not self._is_closing:
            self._send_grasp(self._close_width)
            self._is_closing = True
        elif command > self._toggle_threshold and (not is_open) and self._is_closing:
            self._send_grasp(self._open_width)
            self._is_closing = False

    def _send_grasp(self, width: float):
        goal = Grasp.Goal()
        goal.width = width
        goal.speed = self._gripper_speed
        goal.force = self._gripper_force
        goal.epsilon.inner = self._epsilon_inner
        goal.epsilon.outer = self._epsilon_outer
        self._grasp_client.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)
    node = CrispPyFrankaHandAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
