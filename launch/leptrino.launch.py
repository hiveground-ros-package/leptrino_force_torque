from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    com_port = DeclareLaunchArgument(name="comport", default_value="/dev/ttyUSB0")

    return LaunchDescription([
        com_port,
        Node(package="leptrino_force_torque",
             namespace="leptrino_force_torque",
             executable="leptrino_force_torque",
             output="screen")
    ])
