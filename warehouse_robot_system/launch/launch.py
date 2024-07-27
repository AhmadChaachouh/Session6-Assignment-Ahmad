from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='warehouse_robot_system',
            executable='stock_checker_service_server',
            name='stock_checker_service_server'
        ),
        Node(
            package='warehouse_robot_system',
            executable='item_delivery_action_server',
            name='item_delivery_action_server'
        ),
        Node(
            package='warehouse_robot_system',
            executable='item_delivery_action_client',
            name='item_delivery_action_client'
        ),
        Node(
            package='warehouse_robot_system',
            executable='stock_checker_service_client',
            name='stock_checker_service_client'
        ),
    ])
