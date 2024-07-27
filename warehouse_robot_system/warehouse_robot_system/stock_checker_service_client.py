import rclpy
from rclpy.node import Node
from item_delivery_interface.srv import CheckStock
import matplotlib.pyplot as plt

class StockCheckerServiceClient(Node):
    def __init__(self):
        super().__init__('stock_checker_service_client')
        self.client = self.create_client(CheckStock, 'check_stock')
        self.items = ['item1', 'item2', 'item3']  # Example items

    def send_request(self, item_name):
        request = CheckStock.Request()
        request.item_name = item_name

        self.client.wait_for_service()
        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def get_stock_levels(self):
        stock_levels = {}
        for item in self.items:
            response = self.send_request(item)
            stock_levels[item] = response.stock_level
        return stock_levels

    def plot_stock_levels(self, stock_levels):
        items = list(stock_levels.keys())
        levels = list(stock_levels.values())

        plt.bar(items, levels)
        plt.xlabel('Items')
        plt.ylabel('Stock Levels')
        plt.title('Stock Levels of Items')
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    client = StockCheckerServiceClient()
    stock_levels = client.get_stock_levels()
    print(stock_levels)
    client.plot_stock_levels(stock_levels)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
