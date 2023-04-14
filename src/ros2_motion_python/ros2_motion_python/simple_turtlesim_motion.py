# This file is part of ros2_motion_python package.
#
# Copyright (c) 2023 Anis Koubaa.
# All rights reserved.
#
# This work is licensed under the terms of the Creative Commons Attribution-NonCommercial-ShareAlike 4.0
# International Public License. See https://creativecommons.org/licenses/by-nc-sa/4.0/ for details.
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist 
import time
from std_srvs.srv import Empty

class TurtlesimMover(Node):

    LINEAR_SPEED = 0.2
    ANGULAR_SPEED = 0.3

    def __init__(self):
        super().__init__('turtlesim_mover')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter=0
        self.t0=time.time()
        
        self.client = self.create_client(Empty, 'reset')
        self.request = Empty.Request()
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        print ('Turtlesim Reset')

    def timer_callback(self):
        msg=Twist()

        #msg.data = 'Hello World: %d' %self.counter

        msg.linear.x=TurtlesimMover.LINEAR_SPEED
        msg.linear.y=0.0
        msg.linear.z=0.0

        msg.angular.x=0.0
        msg.angular.y=0.0
        msg.angular.z=TurtlesimMover.ANGULAR_SPEED

        t1 = time.time()
        time_spent = t1-self.t0
        traveled_distance = time_spent * msg.linear.x


        print('Time Spent: ',time_spent)
        print('Traveled Distance: ',traveled_distance)
        if (traveled_distance>15.0):
            msg.linear.x=0.0
            msg.angular.z=0.0 
            self.publisher.publish(msg)
            print('The Robot Has Stopped')
            rclpy.shutdown() 


        #if (time_spent>5.0):
        #    msg.linear.x=0.0
        #    msg.angular.z=0.0
        #    self.publisher.publish(msg)
        #    print('The Robot Has Stopped')
        #    rclpy.shutdown()

        self.publisher.publish(msg)
        self.counter=self.counter+1
        #print('Publishing" ', msg.data, '')
        #self.get_logger().info('Publishing: "%s"' % msg)

def main (args=None):

        rclpy.init(args=args)
        turtlesim_mover = TurtlesimMover()

        rclpy.spin(turtlesim_mover)

        turtlesim_mover.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

        