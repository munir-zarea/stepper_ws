from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    stepper_port = LaunchConfiguration('stepper_port')
    servo_port   = LaunchConfiguration('servo_port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'stepper_port',
            default_value='/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_34336313537351A08121-if00',
            description='Stable /dev/serial/by-id path for the stepper Arduino'
        ),
        DeclareLaunchArgument(
            'servo_port',
            default_value='/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_0353638323635111C1B1-if00',
            description='Stable /dev/serial/by-id path for the servo Arduino'
        ),

        # ====== Joy node ======
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'deadzone': 0.15,
                'autorepeat_rate': 20.0
            }]
        ),

        # ====== Steppers (Arduino #1) ======
        Node(
            package='stepper_joy',
            executable='stepper_joy_node',
            name='stepper_joy_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'port': stepper_port,
                'baud': 115200,

                'axis_x': 0,
                'axis_y': 1,
                'deadband': 0.15,

                'btn_step4_pos': 5,  # RB
                'btn_step4_neg': 4,  # LB
                'step4_steps_per_tick': 5.0,
                'step4_home_steps': 0,
                'step4_max_travel_steps': 2000,

                'movespeed_deg_per_tick': 0.3,
                'update_rate_hz': 60.0,
                'steps_per_rev_effective': 3200,
            }]
        ),

        # ====== Servos (Arduino #2) ======
        Node(
            package='stepper_joy',
            executable='joystick_servo6_serial_node',
            name='joystick_servo6_serial_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'port': servo_port,
                'baud': 115200,

                'axis_rx': 3,
                'axis_ry': 4,
                'axis_lt': 2,
                'axis_rt': 5,

                'update_rate_hz': 100.0,
                'deadband_stick': 0.15,
                'deadband_trig': 0.05,

                'pouch_step_deg_per_tick': 2,
                'default_pos': 150,

                'prop_step_held': 8,
                'prop_step_relax': 6,
                'grip_step_held': 8,
                'grip_step_relax': 6,

                'trig_threshold': 0.10,
            }]
        ),
    ])
