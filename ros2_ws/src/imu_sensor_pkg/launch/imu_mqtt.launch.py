"""
Launch file for IMU sensor package - MQTT/Wireless mode with Lifecycle Management

This launch file starts:
1. MQTT Lifecycle Bridge Node - Receives IMU data from MQTT broker and publishes to ROS2 topics
2. Database Node - Subscribes to ROS2 topics and stores data in SQLite
3. Visualizer Node - Creates arrow visualization for RViz
4. RViz2 - For visualization

The lifecycle node is automatically configured and activated on launch.

Usage:
    ros2 launch imu_sensor_pkg imu_mqtt.launch.py

With custom MQTT broker:
    ros2 launch imu_sensor_pkg imu_mqtt.launch.py mqtt_broker:=192.168.1.100 mqtt_port:=1883
"""

from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    # Declare launch arguments
    mqtt_broker_arg = DeclareLaunchArgument(
        'mqtt_broker',
        default_value='localhost',
        description='MQTT broker hostname or IP address'
    )
    
    mqtt_port_arg = DeclareLaunchArgument(
        'mqtt_port',
        default_value='1883',
        description='MQTT broker port'
    )
    
    mqtt_topic_arg = DeclareLaunchArgument(
        'mqtt_topic',
        default_value='esp32/imu/data',
        description='MQTT topic to subscribe to'
    )
    
    db_path_arg = DeclareLaunchArgument(
        'db_path',
        default_value='/tmp/imu_data.db',
        description='Path to SQLite database file'
    )
    
    # MQTT Lifecycle Bridge Node
    mqtt_bridge_node = LifecycleNode(
        package='imu_sensor_pkg',
        executable='mqtt_imu_bridge',
        name='imu_mqtt_bridge',
        namespace='',
        parameters=[{
            'mqtt_broker': LaunchConfiguration('mqtt_broker'),
            'mqtt_port': LaunchConfiguration('mqtt_port'),
            'mqtt_topic': LaunchConfiguration('mqtt_topic'),
        }],
        output='screen'
    )
    
    # Database Subscriber Node
    database_node = Node(
        package='imu_sensor_pkg',
        executable='imu_database_node',
        name='imu_database',
        output='screen',
        parameters=[{
            'database_path': LaunchConfiguration('db_path'),
        }]
    )
    
    # Visualizer node (voor de pijl in RViz)
    visualizer_node = Node(
        package='imu_sensor_pkg',
        executable='imu_vector_visualizer',
        name='imu_visualizer',
        output='screen'
    )

    # RViz2 node, met configuratiebestand
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
            target_lifecycle_node=mqtt_bridge_node,
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
        mqtt_broker_arg,
        mqtt_port_arg,
        mqtt_topic_arg,
        db_path_arg,
        mqtt_bridge_node,
        database_node,
        visualizer_node,
        rviz_node,
        TimerAction(
            period=2.0,
            actions=[configure_trans_event]
        ),
        activate_trans_event
    ])
