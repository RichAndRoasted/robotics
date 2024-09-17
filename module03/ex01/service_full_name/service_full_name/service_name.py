from names.srv import SummFullName

import rclpy
from rclpy.node import Node


class FullNameService(Node):

    def __init__(self):
        super().__init__('service_name')
        self.srv = self.create_service(SummFullName, 'summ_full_name', self.summ_full_name_callback)

    def summ_full_name_callback(self, request, response):
        response.full_name = request.last_name + " " + request.first_name + " " + request.patronymic
        self.get_logger().info('Incoming request\nLast name: %s \n First name: %s \n Patronymic: %s' % 
                               (request.last_name, request.first_name, request.patronymic))
        return response


def main():
    rclpy.init()

    service_name = FullNameService()

    rclpy.spin(service_name)

    rclpy.shutdown()


if __name__ == '__main__':
    main()