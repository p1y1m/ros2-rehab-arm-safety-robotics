#!/usr/bin/env python3

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64


class RehabExercise(Node):
    UPDATE_PERIOD = 0.05
    START_DURATION = 3.0
    MOVE_DURATION = 6.0
    PAUSE_DURATION = 1.5

    START = (0.0, 0.0)
    LOW = (-0.6, 0.2)
    HIGH = (0.6, 1.0)

    def __init__(self):
        super().__init__("rehab_exercise")
        self.shoulder_publisher = self.create_publisher(
            Float64, "/shoulder_position", 10
        )
        self.elbow_publisher = self.create_publisher(
            Float64, "/elbow_position", 10
        )

        self.start_position = self.START
        self.target_position = self.LOW
        self.move_duration = self.START_DURATION
        self.phase = "move"
        self.phase_started = self.get_clock().now()
        self.create_timer(self.UPDATE_PERIOD, self.update)

    def publish_positions(self, shoulder, elbow):
        self.shoulder_publisher.publish(Float64(data=shoulder))
        self.elbow_publisher.publish(Float64(data=elbow))

    def update(self):
        now = self.get_clock().now()
        elapsed = (now - self.phase_started).nanoseconds / 1e9

        if self.phase == "pause":
            self.publish_positions(*self.target_position)
            if elapsed >= self.PAUSE_DURATION:
                self.start_position = self.target_position
                self.target_position = (
                    self.HIGH
                    if self.target_position == self.LOW
                    else self.LOW
                )
                self.move_duration = self.MOVE_DURATION
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
            self.phase = "pause"
            self.phase_started = now


def main(args=None):
    rclpy.init(args=args)
    node = RehabExercise()
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
