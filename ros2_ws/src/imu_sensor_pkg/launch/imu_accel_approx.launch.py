from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_sensor_pkg',
            executable='accel_velocity_approximator',
            name='accel_velocity_approximator',
            output='screen',
            parameters=[{
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_link',
                'remove_gravity_z': True,
                'use_orientation_for_gravity': False,
                'gravity_m_s2': 9.81,
                'accel_smoothing_alpha': 0.2,
                'accel_deadband': 0.02,
                'velocity_damping': 0.02,
                'max_dt_seconds': 0.2,
            }]
        )
    ])
