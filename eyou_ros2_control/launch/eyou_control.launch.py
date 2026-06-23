from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_control = FindPackageShare("eyou_ros2_control")

    urdf_path = PathJoinSubstitution([pkg_control, "urdf", "eyou_motor.urdf"])
    # 读取URDF文本，强制字符串类型
    robot_description_content = Command(["cat", " ", urdf_path])
    robot_description_param = ParameterValue(robot_description_content, value_type=str)

    ctrl_yaml = PathJoinSubstitution([pkg_control, "config", "eyou_controllers.yaml"])

    # robot_state_publisher
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description_param}],
        output="screen"
    )

    # ros2_control manager
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ctrl_yaml, {"robot_description": robot_description_param}],
        output="screen"
    )

    # 加载控制器
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "joint_state_broadcaster"],
        output="screen"
    )
    load_left_arm_ctrl = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "left_arm_controller"],
        output="screen"
    )
    load_right_arm_ctrl = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "right_arm_controller"],
        output="screen"
    )

    return LaunchDescription([
        robot_state_pub,
        controller_manager,
        load_joint_state_broadcaster,
        load_left_arm_ctrl,
        load_right_arm_ctrl
    ])
