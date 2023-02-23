import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class MyNode(Node):

    turtle_x = 0
    turtle_y = 0 
    turtle_theta = 0

    target_x = 8
    target_y = 5.544445
    target_theta = 0

    kp = 1
    
    def __init__(self, node_name):
        super().__init__(node_name)
        self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)  # publish Twist message to cmd_vel topic
        timer_delay = 0.1
        self.timer = self.create_timer(timer_delay, self.timer_callback)  # publisher callback executes every one second
        self.subscription = self.create_subscription(Pose, "/turtle1/pose", self.listener_callback, 10)    # listen to color sensor topic for Color message
    
    def timer_callback(self):
        # initialize message and change linear/angular velocity
        msg = Twist()
        # msg.linear.x = 2.0
        # msg.angular.z = 1.0

        error_x = self.turtle_x - self.target_x
        print(error_x)
        msg.linear.x = -self.kp * error_x

        # print(f"Message Received - x: {self.turtle_x}, y: {self.turtle_y}, theta: {self.turtle_theta}")
        
        self.publisher_.publish(msg)    # publish the message
    
    def listener_callback(self, msg):
        self.turtle_x = msg.x
        self.turtle_y = msg.y
        self.turtle_theta = msg.theta
        # self.get_logger().info(f"Message Received - x: {msg.x}, y: {msg.y}, theta: {msg.theta}")  # output message to console
        
def main(args=None):
    rclpy.init(args=args)
    
    # create node and spin
    my_node = MyNode("pub_sub_node")
    rclpy.spin(my_node)
    
    # destory node and shut down application
    my_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()