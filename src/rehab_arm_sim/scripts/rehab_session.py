#!/usr/bin/env python3

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger

from safety_monitor import SafetyMonitor


class RehabSession(Node):
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    COMPLETED = "COMPLETED"
    STOPPED = "STOPPED"
    FAULT = "FAULT"

    UPDATE_PERIOD = 0.05
    START_DURATION = 3.0
    MOVE_DURATION = 6.0
    ENDPOINT_PAUSE = 1.5

    SHOULDER_LIMITS = (-1.57, 1.57)
    ELBOW_LIMITS = (0.0, 2.35)
    SHOULDER_VELOCITY_LIMIT = 1.0
    ELBOW_VELOCITY_LIMIT = 1.2

    def __init__(self):
        super().__init__("rehab_session")
        self.declare_parameter("repetitions", 5)
        self.declare_parameter("speed", 1.0)
        self.declare_parameter("shoulder_min", -0.6)
        self.declare_parameter("shoulder_max", 0.6)
        self.declare_parameter("elbow_min", 0.2)
        self.declare_parameter("elbow_max", 1.0)
        self.declare_parameter("shoulder_safe_min", -0.8)
        self.declare_parameter("shoulder_safe_max", 0.8)
        self.declare_parameter("elbow_safe_min", 0.0)
        self.declare_parameter("elbow_safe_max", 1.2)
        self.declare_parameter("max_tracking_error", 0.25)
        self.declare_parameter("allowed_error_duration", 1.0)

        self.repetitions = self.get_parameter("repetitions").value
        self.speed = self.get_parameter("speed").value
        self.low = (
            self.get_parameter("shoulder_min").value,
            self.get_parameter("elbow_min").value,
        )
        self.high = (
            self.get_parameter("shoulder_max").value,
            self.get_parameter("elbow_max").value,
        )
        self.shoulder_safe_limits = (
            self.get_parameter("shoulder_safe_min").value,
            self.get_parameter("shoulder_safe_max").value,
        )
        self.elbow_safe_limits = (
            self.get_parameter("elbow_safe_min").value,
            self.get_parameter("elbow_safe_max").value,
        )
        self.max_tracking_error = self.get_parameter(
            "max_tracking_error"
        ).value
        self.allowed_error_duration = self.get_parameter(
            "allowed_error_duration"
        ).value
        self.validate_configuration()
        self.safety_monitor = SafetyMonitor(
            self.shoulder_safe_limits,
            self.elbow_safe_limits,
            self.max_tracking_error,
            self.allowed_error_duration,
        )

        self.shoulder_command_publisher = self.create_publisher(
            Float64, "/rehab_session/shoulder_command", 10
        )
        self.elbow_command_publisher = self.create_publisher(
            Float64, "/rehab_session/elbow_command", 10
        )
        self.state_publisher = self.create_publisher(
            String, "/rehab_session/state", 10
        )
        self.create_subscription(
            JointState, "/joint_states", self.on_joint_state, 10
        )

        self.create_service(
            Trigger, "/rehab_session/start", self.start_session
        )
        self.create_service(
            Trigger, "/rehab_session/pause", self.pause_session
        )
        self.create_service(
            Trigger, "/rehab_session/resume", self.resume_session
        )
        self.create_service(
            Trigger, "/rehab_session/stop", self.stop_session
        )
        self.create_service(
            Trigger, "/rehab_session/reset_fault", self.reset_fault
        )

        self.state = self.IDLE
        self.phase = "move"
        self.phase_started = self.get_clock().now()
        self.pause_started = None
        self.start_position = (0.0, 0.0)
        self.target_position = self.low
        self.current_position = (0.0, 0.0)
        self.actual_position = None
        self.move_duration = self.START_DURATION / self.speed
        self.completed_repetitions = 0

        self.create_timer(self.UPDATE_PERIOD, self.update)
        self.state_publisher.publish(String(data=self.state))
        self.get_logger().info(
            "state IDLE; call /rehab_session/start to begin"
        )

    def validate_configuration(self):
        if self.repetitions < 1:
            raise ValueError("repetitions must be at least 1")
        if self.speed <= 0.0:
            raise ValueError("speed must be greater than 0")
        if not (
            self.SHOULDER_LIMITS[0]
            <= self.low[0]
            < self.high[0]
            <= self.SHOULDER_LIMITS[1]
        ):
            raise ValueError("shoulder angles exceed the URDF limits")
        if not (
            self.ELBOW_LIMITS[0]
            <= self.low[1]
            < self.high[1]
            <= self.ELBOW_LIMITS[1]
        ):
            raise ValueError("elbow angles exceed the URDF limits")
        if not (
            self.SHOULDER_LIMITS[0]
            <= self.shoulder_safe_limits[0]
            < self.shoulder_safe_limits[1]
            <= self.SHOULDER_LIMITS[1]
        ):
            raise ValueError("shoulder safety range exceeds URDF limits")
        if not (
            self.ELBOW_LIMITS[0]
            <= self.elbow_safe_limits[0]
            < self.elbow_safe_limits[1]
            <= self.ELBOW_LIMITS[1]
        ):
            raise ValueError("elbow safety range exceeds URDF limits")
        if not (
            self.shoulder_safe_limits[0]
            <= self.low[0]
            < self.high[0]
            <= self.shoulder_safe_limits[1]
        ):
            raise ValueError("shoulder exercise exceeds safety limits")
        if not (
            self.elbow_safe_limits[0]
            <= self.low[1]
            < self.high[1]
            <= self.elbow_safe_limits[1]
        ):
            raise ValueError("elbow exercise exceeds safety limits")
        if self.max_tracking_error <= 0.0:
            raise ValueError("max_tracking_error must be greater than 0")
        if self.allowed_error_duration < 0.0:
            raise ValueError(
                "allowed_error_duration cannot be negative"
            )

        move_duration = self.MOVE_DURATION / self.speed
        start_duration = self.START_DURATION / self.speed
        shoulder_peak_speed = max(
            (self.high[0] - self.low[0]) * math.pi / (2 * move_duration),
            abs(self.low[0]) * math.pi / (2 * start_duration),
        )
        elbow_peak_speed = max(
            (self.high[1] - self.low[1]) * math.pi / (2 * move_duration),
            abs(self.low[1]) * math.pi / (2 * start_duration),
        )
        if shoulder_peak_speed > self.SHOULDER_VELOCITY_LIMIT:
            raise ValueError("speed exceeds the shoulder velocity limit")
        if elbow_peak_speed > self.ELBOW_VELOCITY_LIMIT:
            raise ValueError("speed exceeds the elbow velocity limit")

    def transition_to(self, new_state, reason):
        old_state = self.state
        self.state = new_state
        self.state_publisher.publish(String(data=new_state))
        self.get_logger().info(
            "state %s -> %s: %s" % (old_state, new_state, reason)
        )

    def start_session(self, request, response):
        del request
        if self.state != self.IDLE:
            response.success = False
            response.message = "a new session requires relaunching"
            return response

        self.completed_repetitions = 0
        self.safety_monitor.reset()
        self.start_position = self.current_position
        self.target_position = self.low
        self.move_duration = self.START_DURATION / self.speed
        self.phase = "move"
        self.phase_started = self.get_clock().now()
        self.pause_started = None
        self.transition_to(
            self.RUNNING,
            "%d repetitions, speed %.2fx, shoulder %.2f..%.2f, "
            "elbow %.2f..%.2f"
            % (
                self.repetitions,
                self.speed,
                self.low[0],
                self.high[0],
                self.low[1],
                self.high[1],
            ),
        )
        response.success = True
        response.message = "session started"
        return response

    def pause_session(self, request, response):
        del request
        if self.state != self.RUNNING:
            response.success = False
            response.message = "session is not running"
            return response
        self.pause_started = self.get_clock().now()
        self.transition_to(self.PAUSED, "pause requested")
        response.success = True
        response.message = "session paused"
        return response

    def resume_session(self, request, response):
        del request
        if self.state != self.PAUSED:
            response.success = False
            response.message = "session is not paused"
            return response
        now = self.get_clock().now()
        self.phase_started = self.phase_started + (now - self.pause_started)
        self.pause_started = None
        self.transition_to(self.RUNNING, "resume requested")
        response.success = True
        response.message = "session resumed"
        return response

    def stop_session(self, request, response):
        del request
        if self.state not in (self.RUNNING, self.PAUSED):
            response.success = False
            response.message = "no active session to stop"
            return response
        self.transition_to(
            self.STOPPED,
            "stopped after %d/%d repetitions"
            % (self.completed_repetitions, self.repetitions),
        )
        response.success = True
        response.message = "session stopped"
        return response

    def reset_fault(self, request, response):
        del request
        if self.state != self.FAULT:
            response.success = False
            response.message = "session is not in FAULT"
            return response
        self.safety_monitor.reset()
        self.transition_to(self.IDLE, "fault reset requested")
        response.success = True
        response.message = "fault reset; session is IDLE"
        return response

    def on_joint_state(self, message):
        positions = dict(zip(message.name, message.position))
        if (
            "shoulder_joint" not in positions
            or "elbow_joint" not in positions
        ):
            return
        self.actual_position = (
            positions["shoulder_joint"],
            positions["elbow_joint"],
        )
        if self.state not in (self.IDLE, self.RUNNING, self.PAUSED):
            return
        now_seconds = self.get_clock().now().nanoseconds / 1e9
        reason = self.safety_monitor.check(
            self.actual_position,
            self.current_position,
            now_seconds,
            check_tracking_error=self.state in (
                self.RUNNING,
                self.PAUSED,
            ),
        )
        if reason is not None:
            self.transition_to(self.FAULT, reason)

    def publish_positions(self, shoulder, elbow):
        self.current_position = (shoulder, elbow)
        self.shoulder_command_publisher.publish(Float64(data=shoulder))
        self.elbow_command_publisher.publish(Float64(data=elbow))

    def update(self):
        if self.state == self.PAUSED:
            self.publish_positions(*self.current_position)
            return
        if self.state != self.RUNNING:
            return

        now = self.get_clock().now()
        elapsed = (now - self.phase_started).nanoseconds / 1e9

        if self.phase == "pause":
            self.publish_positions(*self.target_position)
            if elapsed < self.ENDPOINT_PAUSE:
                return
            if (
                self.target_position == self.low
                and self.completed_repetitions >= self.repetitions
            ):
                self.transition_to(
                    self.COMPLETED,
                    "%d/%d repetitions completed"
                    % (self.completed_repetitions, self.repetitions),
                )
                return
            self.start_position = self.target_position
            self.target_position = (
                self.high
                if self.target_position == self.low
                else self.low
            )
            self.move_duration = self.MOVE_DURATION / self.speed
            self.phase = "move"
            self.phase_started = now
            return

        progress = min(max(elapsed / self.move_duration, 0.0), 1.0)
        blend = 0.5 - 0.5 * math.cos(math.pi * progress)
        shoulder = self.start_position[0] + blend * (
            self.target_position[0] - self.start_position[0]
        )
        elbow = self.start_position[1] + blend * (
            self.target_position[1] - self.start_position[1]
        )
        self.publish_positions(shoulder, elbow)

        if progress >= 1.0:
            if (
                self.start_position == self.high
                and self.target_position == self.low
            ):
                self.completed_repetitions += 1
                self.get_logger().info(
                    "progress %d/%d repetitions"
                    % (self.completed_repetitions, self.repetitions)
                )
            self.phase = "pause"
            self.phase_started = now


def main(args=None):
    rclpy.init(args=args)
    node = RehabSession()
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
