import sys

from names.srv import SummFullName
import rclpy
from rclpy.node import Node


class NameClientAsync(Node):

    def __init__(self):
        super().__init__('client_name')
        self.cli = self.create_client(SummFullName, 'summ_full_name')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SummFullName.Request()

    def send_request(self, last_name, first_name, patronymic):
        self.req.last_name = last_name
        self.req.first_name = first_name
        self.req.patronymic = patronymic
        return self.cli.call_async(self.req)


def main():
    rclpy.init()

    client_name = NameClientAsync()
    future = client_name.send_request(str(sys.argv[1]), str(sys.argv[2]), str(sys.argv[3]))
    rclpy.spin_until_future_complete(client_name, future)
    response = future.result()
    client_name.get_logger().info(
        'Result of summ_full_name: for %s + %s  + %s = %s' %
        (str(sys.argv[1]), str(sys.argv[2]), str(sys.argv[3]), response.full_name))

    client_name.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()