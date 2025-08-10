from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mqtt_publisher_cpp',
            executable='mqtt_publisher_node',   # <-- nama executable yg benar
            name='mqtt_publisher_node',
            output='screen',
            parameters=[{
                'broker_host': '127.0.0.1',
                'broker_port': 1883,
                'username': 'factory:mqtt',
                'password': 'mqttpass',
                'topic_base': 'site',
                'robot_id': 'robot1',
                'qos': 0,
                'retain': False,
                'acc_free.ros_topic': '/asv/imu/acc_free',
                'acc_free.throttle_hz': 20.0,
                'rpy_deg.ros_topic': '/asv/imu/rpy_deg',
                'rpy_deg.throttle_hz': 10.0,
            }]
        )
    ])
