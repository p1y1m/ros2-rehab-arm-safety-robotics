from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("rehab_arm_sim"))
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))
    world = package_share / "worlds" / "rehab_room.sdf"
    urdf = package_share / "urdf" / "rehab_arm.urdf"
    robot_description = urdf.read_text()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ros_gz_share / "launch" / "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r -s {world}"}.items(),
    )

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": True}
        ],
        output="screen",
    )

    spawn_arm = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "rehab_arm",
            "-z",
            "0.05",
        ],
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    joint_state_topic = "/world/rehab_room/model/rehab_arm/joint_state"
    joint_state_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            f"{joint_state_topic}@sensor_msgs/msg/JointState[gz.msgs.Model"
        ],
        remappings=[(joint_state_topic, "/joint_states")],
        output="screen",
    )

    command_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/shoulder_position@std_msgs/msg/Float64]gz.msgs.Double",
            "/elbow_position@std_msgs/msg/Float64]gz.msgs.Double",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            gazebo,
            state_publisher,
            spawn_arm,
            clock_bridge,
            joint_state_bridge,
            command_bridge,
        ]
    )
