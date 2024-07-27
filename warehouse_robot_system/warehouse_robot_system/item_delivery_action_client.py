import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from item_delivery_interface.action import DeliverItem

class ItemDeliveryActionClient(Node):
    def __init__(self):
        super().__init__('item_delivery_action_client')
        self._action_client = ActionClient(self, DeliverItem, 'deliver_item')

    def send_goal(self, item_name, quantity):
        goal_msg = DeliverItem.Goal()
        goal_msg.item_name = item_name
        goal_msg.quantity = quantity

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback: {feedback_msg.feedback.status}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.success}, {result.message}')

def main(args=None):
    rclpy.init(args=args)
    action_client = ItemDeliveryActionClient()
    item_name = 'item1'  # Example item
    quantity = 3  # Example quantity
    action_client.send_goal(item_name, quantity)
    rclpy.spin(action_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
