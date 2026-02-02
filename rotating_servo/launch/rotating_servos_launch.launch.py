from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rotating_servo',
            executable='rotating_servo_x',
            name='rotating_servo_x_node',
            output='screen',
            emulate_tty=True,
        ),
         Node(
            package='rotating_servo',
            executable='rotating_servo_y',
            name='rotating_servo_y_node',
            output='screen',
            emulate_tty=True,
        )
    ])