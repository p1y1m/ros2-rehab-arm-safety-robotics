from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory("rehab_arm_sim"))
    results_file = "/workspace/rehab_arm_exercise_results.csv"

    repetitions = LaunchConfiguration("repetitions")
    speed = LaunchConfiguration("speed")
    shoulder_min = LaunchConfiguration("shoulder_min")
    shoulder_max = LaunchConfiguration("shoulder_max")
    elbow_min = LaunchConfiguration("elbow_min")
    elbow_max = LaunchConfiguration("elbow_max")
    shoulder_safe_min = LaunchConfiguration("shoulder_safe_min")
    shoulder_safe_max = LaunchConfiguration("shoulder_safe_max")
    elbow_safe_min = LaunchConfiguration("elbow_safe_min")
    elbow_safe_max = LaunchConfiguration("elbow_safe_max")
    max_tracking_error = LaunchConfiguration("max_tracking_error")
    allowed_error_duration = LaunchConfiguration("allowed_error_duration")
    command_safety_margin = LaunchConfiguration("command_safety_margin")
    max_command_rate = LaunchConfiguration("max_command_rate")

    configuration = {
        "use_sim_time": True,
        "repetitions": ParameterValue(repetitions, value_type=int),
        "speed": ParameterValue(speed, value_type=float),
        "shoulder_min": ParameterValue(shoulder_min, value_type=float),
        "shoulder_max": ParameterValue(shoulder_max, value_type=float),
        "elbow_min": ParameterValue(elbow_min, value_type=float),
        "elbow_max": ParameterValue(elbow_max, value_type=float),
        "shoulder_safe_min": ParameterValue(
            shoulder_safe_min, value_type=float
        ),
        "shoulder_safe_max": ParameterValue(
            shoulder_safe_max, value_type=float
        ),
        "elbow_safe_min": ParameterValue(
            elbow_safe_min, value_type=float
        ),
        "elbow_safe_max": ParameterValue(
            elbow_safe_max, value_type=float
        ),
        "max_tracking_error": ParameterValue(
            max_tracking_error, value_type=float
        ),
        "allowed_error_duration": ParameterValue(
            allowed_error_duration, value_type=float
        ),
    }

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(package_share / "launch" / "sim.launch.py")
        )
    )

    session = Node(
        package="rehab_arm_sim",
        executable="rehab_session",
        parameters=[configuration],
        output="screen",
    )

    safety_gate = Node(
        package="rehab_arm_sim",
        executable="safety_command_gate",
        parameters=[
            {
                "use_sim_time": True,
                "shoulder_safe_min": ParameterValue(
                    shoulder_safe_min, value_type=float
                ),
                "shoulder_safe_max": ParameterValue(
                    shoulder_safe_max, value_type=float
                ),
                "elbow_safe_min": ParameterValue(
                    elbow_safe_min, value_type=float
                ),
                "elbow_safe_max": ParameterValue(
                    elbow_safe_max, value_type=float
                ),
                "command_safety_margin": ParameterValue(
                    command_safety_margin, value_type=float
                ),
                "max_command_rate": ParameterValue(
                    max_command_rate, value_type=float
                ),
            }
        ],
        output="screen",
    )

    metrics = Node(
        package="rehab_arm_sim",
        executable="exercise_metrics",
        parameters=[
            {
                "use_sim_time": True,
                "results_file": results_file,
                "shoulder_min": ParameterValue(
                    shoulder_min, value_type=float
                ),
                "shoulder_max": ParameterValue(
                    shoulder_max, value_type=float
                ),
                "elbow_min": ParameterValue(elbow_min, value_type=float),
                "elbow_max": ParameterValue(elbow_max, value_type=float),
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("repetitions", default_value="5"),
            DeclareLaunchArgument("speed", default_value="1.0"),
            DeclareLaunchArgument("shoulder_min", default_value="-0.6"),
            DeclareLaunchArgument("shoulder_max", default_value="0.6"),
            DeclareLaunchArgument("elbow_min", default_value="0.2"),
            DeclareLaunchArgument("elbow_max", default_value="1.0"),
            DeclareLaunchArgument(
                "shoulder_safe_min", default_value="-0.8"
            ),
            DeclareLaunchArgument(
                "shoulder_safe_max", default_value="0.8"
            ),
            DeclareLaunchArgument(
                "elbow_safe_min", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "elbow_safe_max", default_value="1.2"
            ),
            DeclareLaunchArgument(
                "max_tracking_error", default_value="0.25"
            ),
            DeclareLaunchArgument(
                "allowed_error_duration", default_value="1.0"
            ),
            DeclareLaunchArgument(
                "command_safety_margin", default_value="0.10"
            ),
            DeclareLaunchArgument(
                "max_command_rate", default_value="0.5"
            ),
            simulation,
            safety_gate,
            session,
            metrics,
        ]
    )
