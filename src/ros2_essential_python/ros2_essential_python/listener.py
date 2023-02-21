import rclpy 
from rclpy.node import Node
from std_msgs.msg import String 

class ListenerSubscriber(Node):

    def __init__(self):
        super().__init__('listen_susbscribe')
        self.subscription = self.create_subscription(
            String, 
            'chatter', 
            self.listen_callback, 
            10)

        #self.subscription #prevent warning of unused variable

    def listen_callback(self, msg):
        #print()
        self.get_logger().info('I heard: "%s"' % msg.data)

def main (args = None):
    rclpy.init(args=args)

    listen_susbscribe = ListenerSubscriber()

    rclpy.spin(listen_susbscribe)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    listen_susbscribe.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()