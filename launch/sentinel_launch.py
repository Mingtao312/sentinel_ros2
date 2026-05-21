from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sentinel_ros',
            executable='sentinel_node',
            name='sentinel_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                # 你可以在这里传 ROS 2 参数，比如分辨率、usb id 等
                # {'usb_vendor_id': 0x04b4, 'usb_product_id': 0x00f1}
            ]
        )
    ])