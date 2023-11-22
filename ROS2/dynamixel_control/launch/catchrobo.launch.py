from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package="micro_ros_agent",
      executable="micro_ros_agent",
      name="micro_ros_agent_serial_acm0",
      namespace="micro_ros_agent_serial_acm0",
      output="screen",
      arguments=["serial", "-b", "115200", "--dev", "/dev/ttyUSB0"]
    ),
    Node(
      package="dynamixel_control",
      executable="dynamixel_node",
      name="dynamixel_node",
      output="screen"
    )
])
