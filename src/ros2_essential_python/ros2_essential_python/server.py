from ros2_interfaces_cpp.srv import AddTwoInts

import rclpy
from rclpy.node import Node
from time import sleep

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        print ('waiting for new requests ...')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        sleep(5)
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        print ('sending response and waiting for new requests ...')
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()