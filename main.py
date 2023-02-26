import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from rcl_interfaces.msg import Parameter
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue

from turtlesim.srv import SetPen

import json
from math import *

class MyNode(Node):
    # variables to track the robot's state
    turtle_x = 0
    turtle_y = 0 
    turtle_theta = 0

    points = []         # list of waypoints for navigation
    current_target = 0  # index of the target point in the list

    rotate = True   # state control variable for rotating vs. translating

    # pid parameters
    rotate_error_thresh = 0.0001
    translate_error_thresh = 0.01
    kp = 5.0
    
    def __init__(self, node_name):
        # get points from json file
        with open("points.json", 'r') as f:
            self.points = json.load(f)

        super().__init__(node_name)

        # change background color
        pen_client = self.create_client(SetPen, '/turtle1/set_pen')

        request = SetPen.Request()
        request.r = 255
        request.g = 255
        request.b = 255
        request.width = 5

        pen_client.call_async(request)


        bg_client = self.create_client(SetParameters, '/turtlesim/set_parameters')

        request = SetParameters.Request()
        request.parameters = []
        request.parameters.append(Parameter(name='background_r', value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=80)))
        request.parameters.append(Parameter(name='background_g', value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=0)))
        request.parameters.append(Parameter(name='background_b', value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=0)))

        bg_client.call_async(request).add_done_callback(self.pub_sub_callback)

    def pub_sub_callback(self, future):
        self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)  # publish Twist message to cmd_vel topic
        timer_delay = 0.1
        self.timer = self.create_timer(timer_delay, self.timer_callback)  # publisher callback executes every one second
        self.subscription = self.create_subscription(Pose, "/turtle1/pose", self.listener_callback, 10)    # listen to color sensor topic for Color message

    def timer_callback(self):
        # leave function if there are no points left
        if self.current_target == len(self.points):
            return

        # initialize message to send
        msg = Twist()

        # get target point
        target = self.points[self.current_target]

        if self.rotate:
            # calculate target angle and set pid
            angle = atan2(target[1] - self.turtle_y, target[0] - self.turtle_x)
            error = self.turtle_theta - angle
            msg.angular.z = -self.kp * error    

            print(f"ERROR: {error}")

            # change state variable to translate next
            if abs(error) < self.rotate_error_thresh:
                print("ending rotation")
                self.rotate = False

        else:
            # calculate target distance and set pid
            error_x = self.turtle_x - target[0]
            error_y = self.turtle_y - target[1]
            error = -((error_x) ** 2 + (error_y) ** 2) ** 0.5
            msg.linear.x = -self.kp * error

            print(f"ERROR: {error}")

            # change state variable to rotate next
            if abs(error) < self.translate_error_thresh:
                print("ending translation")
                self.rotate = True
                self.current_target += 1
        
        self.publisher_.publish(msg)    # publish the message
    
    # set pose of turtle to node's instance variables when messages are published to /cmd_vel
    def listener_callback(self, msg):
        self.turtle_x = msg.x
        self.turtle_y = msg.y
        self.turtle_theta = msg.theta
        
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