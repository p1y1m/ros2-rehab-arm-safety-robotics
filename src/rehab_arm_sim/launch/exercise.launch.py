from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("rehab_arm_sim"))
    results_file = Path("/workspace/rehab_arm_exercise_results.csv")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(package_share / "launch" / "sim.launch.py")
        )
    )

    exercise = Node(
        package="rehab_arm_sim",
        executable="rehab_exercise",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    metrics = Node(
        package="rehab_arm_sim",
        executable="exercise_metrics",
        parameters=[
            {
                "use_sim_time": True,
                "results_file": str(results_file),
            }
        ],
        output="screen",
    )

    return LaunchDescription([simulation, exercise, metrics])
