import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist # topic is of type Twist
from turtlesim.msg import Color # topic uses Color type


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('turtle_draw')
        # create publisher to topic cmd_vel
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # create subscriber to color_sensor
        self.subscription = self.create_subscription(
            Color,
            '/turtle1/color_sensor',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    

    def timer_callback(self):
        msg = Twist()
        # set linear velocity to make turtle go straight
        msg.linear.x = 2.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        # set angular velocity to have slight turn
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = -2.0
        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        # log msg recieved
        self.get_logger().info(f'Color: r: {msg.r}, g: {msg.g}, b: {msg.b}')


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
