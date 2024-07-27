import rclpy
from rclpy.node import Node
from item_delivery_interface.srv import CheckStock

class StockCheckerService(Node):
    def __init__(self):
        super().__init__('stock_checker_service')
        self.srv = self.create_service(CheckStock, 'check_stock', self.check_stock_callback)
        self.stock = {'item1': 10, 'item2': 5, 'item3': 0}  # Example stock levels

    def check_stock_callback(self, request, response):
        item_name = request.item_name
        if item_name in self.stock:
            response.stock_level = self.stock[item_name]
        else:
            response.stock_level = 0  # Default to 0 if item not found
        return response

def main(args=None):
    rclpy.init(args=args)
    node = StockCheckerService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
