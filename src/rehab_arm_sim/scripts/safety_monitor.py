import math


class SafetyMonitor:
    POSITION_TOLERANCE = 0.01

    def __init__(
        self,
        shoulder_limits,
        elbow_limits,
        max_tracking_error,
        allowed_error_duration,
    ):
        self.limits = {
            "shoulder": shoulder_limits,
            "elbow": elbow_limits,
        }
        self.max_tracking_error = max_tracking_error
        self.allowed_error_duration = allowed_error_duration
        self.error_started = {"shoulder": None, "elbow": None}

    def reset(self):
        self.error_started = {"shoulder": None, "elbow": None}

    def check(
        self,
        actual,
        commanded,
        now_seconds,
        check_tracking_error=True,
    ):
        for index, joint in enumerate(("shoulder", "elbow")):
            position = actual[index]
            lower, upper = self.limits[joint]
            if (
                not math.isfinite(position)
                or position < lower - self.POSITION_TOLERANCE
                or position > upper + self.POSITION_TOLERANCE
            ):
                return (
                    f"{joint} actual angle {position:.3f} rad is outside "
                    f"safe range [{lower:.3f}, {upper:.3f}] rad"
                )

        if not check_tracking_error:
            self.error_started = {"shoulder": None, "elbow": None}
            return None

        for index, joint in enumerate(("shoulder", "elbow")):
            error = abs(commanded[index] - actual[index])
            if error <= self.max_tracking_error:
                self.error_started[joint] = None
                continue

            if self.error_started[joint] is None:
                self.error_started[joint] = now_seconds
            excessive_for = max(
                0.0, now_seconds - self.error_started[joint]
            )
            if excessive_for >= self.allowed_error_duration:
                return (
                    f"{joint} tracking error {error:.3f} rad exceeded "
                    f"{self.max_tracking_error:.3f} rad for "
                    f"{excessive_for:.2f} s"
                )

        return None
