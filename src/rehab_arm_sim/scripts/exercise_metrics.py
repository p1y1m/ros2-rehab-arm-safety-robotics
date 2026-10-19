#!/usr/bin/env python3

import csv
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String


class ExerciseMetrics(Node):
    SAMPLE_PERIOD = 0.05
    PRINT_PERIOD = 1.0
    ENDPOINT_TOLERANCE = 0.01

    def __init__(self):
        super().__init__("exercise_metrics")
        self.declare_parameter(
            "results_file", "/workspace/rehab_arm_exercise_results.csv"
        )
        self.declare_parameter("shoulder_min", -0.6)
        self.declare_parameter("shoulder_max", 0.6)
        self.declare_parameter("elbow_min", 0.2)
        self.declare_parameter("elbow_max", 1.0)
        self.results_file = Path(
            self.get_parameter("results_file").value
        ).resolve()
        self.low = (
            self.get_parameter("shoulder_min").value,
            self.get_parameter("elbow_min").value,
        )
        self.high = (
            self.get_parameter("shoulder_max").value,
            self.get_parameter("elbow_max").value,
        )

        self.shoulder_command = None
        self.elbow_command = None
        self.shoulder_actual = None
        self.elbow_actual = None
        self.started_at = None
        self.ended_at = None
        self.high_reached = False
        self.completed_cycles = 0
        self.samples = []
        self.saved = False

        self.create_subscription(
            Float64, "/shoulder_position", self.on_shoulder_command, 10
        )
        self.create_subscription(
            Float64, "/elbow_position", self.on_elbow_command, 10
        )
        self.create_subscription(
            JointState, "/joint_states", self.on_joint_state, 10
        )
        self.create_subscription(
            String, "/rehab_session/state", self.on_session_state, 10
        )
        self.create_timer(self.SAMPLE_PERIOD, self.record_sample)
        self.create_timer(self.PRINT_PERIOD, self.print_metrics)

    def on_shoulder_command(self, message):
        self.shoulder_command = message.data
        self.update_cycle_count()
        self.start_if_ready()

    def on_elbow_command(self, message):
        self.elbow_command = message.data
        self.update_cycle_count()
        self.start_if_ready()

    def on_joint_state(self, message):
        positions = dict(zip(message.name, message.position))
        self.shoulder_actual = positions.get(
            "shoulder_joint", self.shoulder_actual
        )
        self.elbow_actual = positions.get(
            "elbow_joint", self.elbow_actual
        )

    def on_session_state(self, message):
        if message.data not in ("COMPLETED", "STOPPED") or self.saved:
            return
        self.ended_at = self.get_clock().now()
        if self.metrics_ready():
            self.samples.append(self.current_metrics())
        self.save_results()

    def start_if_ready(self):
        if (
            self.started_at is None
            and self.shoulder_command is not None
            and self.elbow_command is not None
        ):
            self.started_at = self.get_clock().now()

    def at_endpoint(self, endpoint):
        if self.shoulder_command is None or self.elbow_command is None:
            return False
        return (
            abs(self.shoulder_command - endpoint[0])
            <= self.ENDPOINT_TOLERANCE
            and abs(self.elbow_command - endpoint[1])
            <= self.ENDPOINT_TOLERANCE
        )

    def update_cycle_count(self):
        if self.at_endpoint(self.high):
            self.high_reached = True
        elif self.high_reached and self.at_endpoint(self.low):
            self.completed_cycles += 1
            self.high_reached = False

    def elapsed_seconds(self):
        if self.started_at is None:
            return 0.0
        end_time = self.ended_at or self.get_clock().now()
        return (
            end_time - self.started_at
        ).nanoseconds / 1e9

    def metrics_ready(self):
        return None not in (
            self.shoulder_command,
            self.elbow_command,
            self.shoulder_actual,
            self.elbow_actual,
        )

    def current_metrics(self):
        return {
            "record_type": "sample",
            "elapsed_s": self.elapsed_seconds(),
            "completed_cycles": self.completed_cycles,
            "shoulder_command_rad": self.shoulder_command,
            "shoulder_actual_rad": self.shoulder_actual,
            "shoulder_abs_error_rad": abs(
                self.shoulder_command - self.shoulder_actual
            ),
            "elbow_command_rad": self.elbow_command,
            "elbow_actual_rad": self.elbow_actual,
            "elbow_abs_error_rad": abs(
                self.elbow_command - self.elbow_actual
            ),
        }

    def record_sample(self):
        if self.ended_at is None and self.metrics_ready():
            self.samples.append(self.current_metrics())

    def print_metrics(self):
        if self.ended_at is not None or not self.metrics_ready():
            return
        metrics = self.current_metrics()
        self.get_logger().info(
            "t=%.1fs cycles=%d | S cmd=%.3f act=%.3f err=%.3f | "
            "E cmd=%.3f act=%.3f err=%.3f"
            % (
                metrics["elapsed_s"],
                metrics["completed_cycles"],
                metrics["shoulder_command_rad"],
                metrics["shoulder_actual_rad"],
                metrics["shoulder_abs_error_rad"],
                metrics["elbow_command_rad"],
                metrics["elbow_actual_rad"],
                metrics["elbow_abs_error_rad"],
            )
        )

    def save_results(self):
        if self.saved:
            return
        self.saved = True

        fieldnames = [
            "record_type",
            "elapsed_s",
            "completed_cycles",
            "shoulder_command_rad",
            "shoulder_actual_rad",
            "shoulder_abs_error_rad",
            "elbow_command_rad",
            "elbow_actual_rad",
            "elbow_abs_error_rad",
        ]
        summary = {
            "record_type": "summary",
            "elapsed_s": self.elapsed_seconds(),
            "completed_cycles": self.completed_cycles,
        }

        self.results_file.parent.mkdir(parents=True, exist_ok=True)
        with self.results_file.open("w", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.samples)
            writer.writerow(summary)

        self.get_logger().info(
            "saved %d samples to %s"
            % (len(self.samples), self.results_file)
        )


def main(args=None):
    rclpy.init(args=args)
    node = ExerciseMetrics()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.save_results()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
