import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class TurtlebotController(Node):

    def __init__(self):
        super().__init__('turtlebot_controller')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.keyboard_sub = self.create_subscription(String, 'keyboard', self.keyboard_callback, 10)
        self.twist = Twist()
        self.obstacle_detected = False
        self.obstacle_distance = 0.0
        self.obstacle_position = ''

        # PID constants for linear and angular velocity control
        self.linear_kp = 0.5
        self.linear_ki = 0.01
        self.linear_kd = 0.1
        self.angular_kp = 0.5
        self.angular_ki = 0.01
        self.angular_kd = 0.1

        # PID variables for linear and angular velocity control
        self.linear_error_sum = 0.0
        self.angular_error_sum = 0.0
        self.last_linear_error = 0.0
        self.last_angular_error = 0.0

        rclpy.spin_once(self)

    def scan_callback(self, msg):
        min_distance = min(msg.ranges)
        if min_distance < 0.5:
            self.obstacle_detected = True
            self.obstacle_distance = min_distance
            min_index = msg.ranges.index(min_distance)
            if min_index < len(msg.ranges)/2:
                self.obstacle_position = 'left'
            else:
                self.obstacle_position = 'right'
        else:
            self.obstacle_detected = False

    def keyboard_callback(self, msg):
        if msg.data == 'q':
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
            self.get_logger().info('Stopping and quitting...')
            rclpy.shutdown()

    def pid_control(self, error, last_error, error_sum, kp, ki, kd):
        proportional = kp * error
        integral = ki * error_sum
        derivative = kd * (error - last_error)
        return proportional + integral + derivative

    def navigate(self):
        print('navigate\n')
        rclpy.spin_once(self)
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.obstacle_detected:
                # Adjust linear velocity based on obstacle distance
                linear_error = 0.5 - self.obstacle_distance
                self.linear_error_sum += linear_error
                linear_output = self.pid_control(linear_error, self.last_linear_error, self.linear_error_sum,
                                                  self.linear_kp, self.linear_ki, self.linear_kd)
                self.last_linear_error = linear_error
                self.twist.linear.x = max(min(0.3, linear_output), 0.0)  # limit the linear velocity to 0.3 m/s

                # Adjust angular velocity based on obstacle position
                if self.obstacle_position == 'left':
                    angular_error = -0.5
                else:
                    angular_error = 0.5
                self.angular_error_sum += angular_error
                angular_output = self.pid_control(angular_error, self.last_angular_error, self.angular_error_sum,
                                                   self.angular_kp, self.angular_ki, self.angular_kd)
                self.twist.angular.z = max(min(0.8, angular_output), 0.0)  # limit the linear velocity to 0.3 m/s
                self.cmd_vel_pub.publish(self.twist)
                

def main(args=None):
    rclpy.init(args=args)
    turtlebot_controller = TurtlebotController()
    turtlebot_controller.navigate()
    rclpy.spin(turtlebot_controller)
    turtlebot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
