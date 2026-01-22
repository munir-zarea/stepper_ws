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

                # joystick mapping
                'axis_x': 0,
                'axis_y': 1,          # unused by new ROS logic (kept for compatibility)
                'deadband': 0.15,
                'invert_x': False,
                'invert_y': True,     # unused by new ROS logic (kept)

                # (kept params; not used for AT anymore in the new ROS code)
                'alpha_deg_per_tick': 0.3,
                'theta_deg_per_tick': 0.3,
                'alpha_min_deg': 0.0,
                'alpha_max_deg': 260.0,
                'theta_offset_deg': 0.0,
                'invert_theta': False,
                'theta_start_deg': 90.0,
                'theta_home_deg': 90.0,
                'alpha_start_deg': 0.0,
                'alpha_home_deg': 0.0,

                # home / deadman (optional)
                'btn_home': 3,               # Y
                'use_deadman': False,
                'btn_deadman': 0,            # A

                'update_rate_hz': 100.0,

                # ===== NEW: Use Arduino legacy P command to move ONLY one of s1/s2/s3 =====
                # Arduino expects: P t1 t2 t3 t4\n  (targets in steps)
                'cmd_p_format': "P %ld %ld %ld %ld\n",

                # Joystick controls s1 (t1) continuously while held left/right
                'joy_stepper_id': 1,          # 1=s1, 2=s2, 3=s3
                'joy_steps_per_tick': 1,     # steps per tick while outside deadband (tune 5-50)

                # Optional initial/home targets for s1/s2/s3 (steps)
                't1_start_steps': 0,
                't2_start_steps': 0,
                't3_start_steps': 0,
                't1_home_steps': 0,
                't2_home_steps': 0,
                't3_home_steps': 0,

                # ===== Stepper4 stays RB/LB via Z (UNCHANGED) =====
                'send_step4': True,
                'cmd_step4_format': "Z %ld\n",
                'btn_step4_pos': 5,          # RB
                'btn_step4_neg': 4,          # LB
                'step4_steps_per_tick': 5.0,
                'step4_home_steps': 0,
                'step4_max_travel_steps': 2000,

                # optional enable command (not required if Arduino AUTO-EN works)
                'send_enable_cmd': False,
                'enable_cmd': "EN 1\n",

                # Slack for opposing stepper motors
                'slack_fraction': 0.05,   # try 0.05 to 0.20
                'slack_min_steps': 0,
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
