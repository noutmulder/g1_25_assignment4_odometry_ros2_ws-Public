from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import EmitEvent, RegisterEventHandler, TimerAction
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Serial IMU Lifecycle Node
    serial_imu_node = LifecycleNode(
        package='imu_sensor_pkg',
        executable='serial_imu_node',
        name='imu_serial_bridge',
        namespace='',
        parameters=[{
            'serial_port': '/dev/ttyACM0',
            'baud_rate': 115200,
            'publish_rate_ms': 10
        }],
        output='screen'
    )

    # Database Subscriber Node
    database_node = Node(
        package='imu_sensor_pkg',
        executable='imu_database_node',
        name='imu_database',
        parameters=[{
            'database_path': 'imu_data.db'
        }],
        output='screen'
    )

    # Visualizer node (voor de pijl in RViz)
    visualizer_node = Node(
        package='imu_sensor_pkg',
        executable='imu_vector_visualizer',
        name='imu_visualizer',
        output='screen'
    )

    # RViz2 node, met jouw configuratiebestand
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('imu_sensor_pkg'),
        'rviz',
        'display.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # Lifecycle transitions
    configure_trans_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: True,
            transition_id=Transition.TRANSITION_CONFIGURE
        )
    )

    activate_trans_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=serial_imu_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda node: True,
                        transition_id=Transition.TRANSITION_ACTIVATE
                    )
                )
            ]
        )
    )

    return LaunchDescription([
        serial_imu_node,
        database_node,
        visualizer_node,
        rviz_node,  # <-- start RViz automatisch
        TimerAction(
            period=2.0,
            actions=[configure_trans_event]
        ),
        activate_trans_event
    ])
