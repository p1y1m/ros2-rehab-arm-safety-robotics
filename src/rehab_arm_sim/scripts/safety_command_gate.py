#!/usr/bin/env python3

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64, String


class SafetyCommandGate(Node):
    HOLD_PERIOD = 0.05

    def __init__(self):
        super().__init__("safety_command_gate")
        self.declare_parameter("shoulder_safe_min", -0.8)
        self.declare_parameter("shoulder_safe_max", 0.8)
        self.declare_parameter("elbow_safe_min", 0.0)
        self.declare_parameter("elbow_safe_max", 1.2)
        self.declare_parameter("command_safety_margin", 0.10)
        self.declare_parameter("max_command_rate", 0.5)

        self.physical_limits = {
            "shoulder": (
                self.get_parameter("shoulder_safe_min").value,
                self.get_parameter("shoulder_safe_max").value,
            ),
            "elbow": (
                self.get_parameter("elbow_safe_min").value,
                self.get_parameter("elbow_safe_max").value,
            ),
        }
        self.command_safety_margin = self.get_parameter(
            "command_safety_margin"
        ).value
        if (
            not math.isfinite(self.command_safety_margin)
            or self.command_safety_margin < 0.0
        ):
            raise ValueError("command_safety_margin must be non-negative")
        self.max_command_rate = self.get_parameter(
            "max_command_rate"
        ).value
        if (
            not math.isfinite(self.max_command_rate)
            or self.max_command_rate <= 0.0
        ):
            raise ValueError("max_command_rate must be greater than 0")

        self.command_limits = {}
        for joint, (lower, upper) in self.physical_limits.items():
            if lower >= upper:
                raise ValueError(f"invalid {joint} safety limits")
            command_lower = lower + self.command_safety_margin
            command_upper = upper - self.command_safety_margin
            if command_lower >= command_upper:
                raise ValueError(
                    f"command_safety_margin is too large for {joint}"
                )
            self.command_limits[joint] = (command_lower, command_upper)

        self.state = "IDLE"
        self.last_safe = {
            joint: min(max(0.0, lower), upper)
            for joint, (lower, upper) in self.command_limits.items()
        }
        self.target_commands = dict(self.last_safe)
        self.command_received = {"shoulder": False, "elbow": False}
        self.last_update_time = self.get_clock().now()
        self.command_publishers = {
            "shoulder": self.create_publisher(
                Float64, "/shoulder_position", 10
            ),
            "elbow": self.create_publisher(
                Float64, "/elbow_position", 10
            ),
        }

        self.create_subscription(
            Float64,
            "/rehab_session/shoulder_command",
            lambda message: self.on_desired("shoulder", message.data),
            10,
        )
        self.create_subscription(
            Float64,
            "/rehab_session/elbow_command",
            lambda message: self.on_desired("elbow", message.data),
            10,
        )
        self.create_subscription(
            String, "/rehab_session/state", self.on_state, 10
        )
        self.create_timer(self.HOLD_PERIOD, self.update_commands)

    def on_state(self, message):
        was_faulted = self.state == "FAULT"
        self.state = message.data
        if self.state == "FAULT" and not was_faulted:
            self.target_commands = dict(self.last_safe)
            self.get_logger().error(
                "FAULT active; holding shoulder %.3f rad and elbow %.3f rad"
                % (self.last_safe["shoulder"], self.last_safe["elbow"])
            )
        elif was_faulted and self.state != "FAULT":
            self.get_logger().info("fault hold released in %s" % self.state)

    def on_desired(self, joint, value):
        lower, upper = self.command_limits[joint]
        if not math.isfinite(value):
            self.get_logger().error(
                "blocked invalid %s desired command: %s" % (joint, value)
            )
            return
        if self.state == "FAULT":
            self.get_logger().warning(
                "blocked %s desired command %.3f rad while in FAULT"
                % (joint, value)
            )
            return

        safe_value = min(max(value, lower), upper)
        if safe_value != value:
            self.get_logger().warning(
                "clamped %s command %.3f -> %.3f rad"
                % (joint, value, safe_value)
            )
        self.target_commands[joint] = safe_value
        self.command_received[joint] = True

    @staticmethod
    def slew_toward(current, target, max_change):
        difference = target - current
        if abs(difference) <= max_change:
            return target
        return current + math.copysign(max_change, difference)

    def update_commands(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = now
        if elapsed <= 0.0:
            return

        if self.state != "FAULT":
            max_change = self.max_command_rate * min(
                elapsed, self.HOLD_PERIOD
            )
            for joint in ("shoulder", "elbow"):
                if not self.command_received[joint]:
                    continue
                self.last_safe[joint] = self.slew_toward(
                    self.last_safe[joint],
                    self.target_commands[joint],
                    max_change,
                )

        for joint in ("shoulder", "elbow"):
            if self.state != "FAULT" and not self.command_received[joint]:
                continue
            self.command_publishers[joint].publish(
                Float64(data=self.last_safe[joint])
            )


def main(args=None):
    rclpy.init(args=args)
    node = SafetyCommandGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
