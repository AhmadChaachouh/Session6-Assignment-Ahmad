import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from item_delivery_interface.action import DeliverItem
from item_delivery_interface.srv import CheckStock

class ItemDeliveryActionServer(Node):
    def __init__(self):
        super().__init__('item_delivery_action_server')
        self.action_server = ActionServer(self, DeliverItem, 'deliver_item', self.execute_callback)
        self.client = self.create_client(CheckStock, 'check_stock')

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing goal: {goal_handle.request.item_name}, Quantity: {goal_handle.request.quantity}')

        # Check stock level
        item_name = goal_handle.request.item_name
        quantity = goal_handle.request.quantity
        request = CheckStock.Request()
        request.item_name = item_name

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Stock service not available, waiting again...')
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        stock_response = future.result()

        if stock_response.stock_level >= quantity:
            feedback_msg = DeliverItem.Feedback()
            feedback_msg.status = f'Delivering {quantity} of {item_name}'
            goal_handle.publish_feedback(feedback_msg)
            
            # Simulate delivery
            self.get_logger().info(f'Delivering {quantity} of {item_name}')
            goal_handle.succeed()
            result = DeliverItem.Result()
            result.success = True
            result.message = f'Successfully delivered {quantity} of {item_name}'
        else:
            self.get_logger().info(f'Insufficient stock for {item_name}')
            goal_handle.abort()
            result = DeliverItem.Result()
            result.success = False
            result.message = f'Failed to deliver {quantity} of {item_name}. Insufficient stock.'
        
        return result

def main(args=None):
    rclpy.init(args=args)
    node = ItemDeliveryActionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
