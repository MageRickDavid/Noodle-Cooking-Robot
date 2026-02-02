from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='servo_controller',
            executable='servo_controller_x',
            name='servo_controller_x',
            output='screen',
            emulate_tty=True,
        ),
         Node(
            package='servo_controller',
            executable='servo_controller_y',
            name='servo_controller_y',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='rotating_servo',
            executable='rotating_servoZ',
            name='rotating_servo_Z',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='rotating_servo',
            executable='rotating_servoX',
            name='rotating_servo_X',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='gripper_controller',
            executable='gripper_controller',
            name='gripper_controller',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='pushers_controller',
            executable='pushers_controller_services',
            name='pusher_controllers',
            output='screen',
            emulate_tty=True,
        ),
        Node(
             package='cscam_control',
             executable='cscam_control',
             name='cscam_control',
             output='screen',
             emulate_tty=True,
         ),
        Node(
             package='cscam_control',
             executable='cscamZ_control',
             name='control_z_motor',
             output='screen',
             emulate_tty=True,
         ),

         Node(
                package='conveyor_controller',
                executable='conveyor_control',
                name='conveyor_controller',
                output='screen',
                emulate_tty=True,
         ),
        Node(
            package='pump_control',
            executable='pump_control',
            name='pump_control',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='sequence',
            executable='sequence_algorithm',
            name='sequence_algorithm',
            output='screen',
            emulate_tty=True,
        ),       
    ])
