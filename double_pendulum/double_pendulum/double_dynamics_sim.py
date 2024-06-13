import numpy as np
import time
from math import sin, cos

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from custom_msgs2.msg import  States

class double_pendulum(Node):
    theta10 = np.pi 
    theta1_dot0 = 0.0
    theta20 = np.pi/3
    theta2_dot0 = 0.0
    
    state_update_frequency = 3000
    state_update_timeperiod = 1 / state_update_frequency

    feedback_frequency = 50

    def __init__(self):
        super().__init__('main')
        self.m1 = 1.5
        self.m2 = 1.0
        self.g = 9.81
        self.l1 = 1.0
        self.l2 = 0.7

        update_states_timer = self.create_timer(1 / self.state_update_frequency, self.update_pendulum_states)
        feedback_timer = self.create_timer(1 / self.feedback_frequency, self.feedback)
        self.visualizer = self.create_publisher(Marker, '/pendulum_viz', 1)
        self.feedback_pub1 = self.create_publisher(States, '/state_feedback1', 1)
        self.feedback_pub2 = self.create_publisher(States, '/state_feedback2', 1)

        self.t_start = time.time()
        self.t_prev = time.time() - 0.0001   
        self.obj_id = 0

        self.theta1 = self.theta10
        self.theta1_dot = self.theta1_dot0
        self.theta2 = self.theta20
        self.theta2_dot = self.theta2_dot0
        self.get_logger().info('Double Inverted Pendulum node initialized')
        self.get_logger().info('Accepting Input')
        self.get_logger().info('Publishing Feedback')

    def update_pendulum_states(self):

        dt = time.time() - self.t_prev
        self.t_prev = time.time()

        theta1_doubledot = (-self.g * (2 * self.m1 + self.m2) * sin(self.theta1) - self.m2 * self.g * sin(self.theta1 - 2 * self.theta2) - 2 * sin(self.theta1 - self.theta2) * self.m2 * (self.theta2_dot ** 2 * self.l2 + self.theta1_dot ** 2 * self.l1 * cos(self.theta1 - self.theta2))) / (self.l1 * (2 * self.m1 + self.m2 - self.m2 * cos(2 * self.theta1 - 2 * self.theta2)))
        theta2_doubledot = (2 * sin(self.theta1 - self.theta2) * (self.theta1_dot ** 2 * self.l1 * (self.m1 + self.m2) + self.g * (self.m1 + self.m2) * cos(self.theta1) + self.theta2_dot ** 2 * self.l2 * self.m2 * cos(self.theta1 - self.theta2))) / (self.l2 * (2 * self.m1 + self.m2 - self.m2 * cos(2 * self.theta1 - 2 * self.theta2)))


        self.theta1_dot += theta1_doubledot * dt
        self.theta2_dot += theta2_doubledot * dt


        self.theta1 += self.theta1_dot * dt
        self.theta2 += self.theta2_dot * dt

        self.visualize_pendulum()
       
        return

    def feedback(self):
        states_msg1 = States()
        states_msg1.theta = self.theta1
        states_msg1.theta_dot = self.theta1_dot
        states_msg2 = States()
        states_msg2.theta = self.theta2
        states_msg2.theta_dot = self.theta2_dot
        self.feedback_pub1.publish(states_msg1)
        self.feedback_pub2.publish(states_msg2)
        return
    
    def visualize_pendulum(self):
        pendulum_marker = Marker()
        pendulum_marker.header.frame_id = "map"
        pendulum_marker.id = 0
        pendulum_marker.type = Marker.LINE_STRIP
        pendulum_marker.action = Marker.ADD
        pendulum_marker.pose.orientation.w = 1.0
        pendulum_marker.scale.x = 0.05


        point_1 = Point()
        point_1.x = 0.0
        point_1.y = 0.0
        point_1.z = 0.0

        point_2 = Point()
        point_2.x = self.l1 * sin(self.theta1)
        point_2.y = - self.l1 * cos(self.theta1)
        point_2.z = 0.0

        point_3 = Point()
        point_3.x = self.l1 * sin(self.theta1)
        point_3.y = - self.l1 * cos(self.theta1)
        point_3.z = 0.0

        point_4 = Point()
        point_4.x = point_3.x + (self.l2 * sin(self.theta2))
        point_4.y =  point_3.y - (self.l2 * cos(self.theta2))
        point_4.z = 0.0
    
        pendulum_marker.points = [point_1,
                            point_2, point_3, point_4
                        ]

        pendulum_marker.color.r = 1.0
        pendulum_marker.color.a = 1.0  
        Duration_of_pendulum_marker = Duration()
        Duration_of_pendulum_marker.sec = 0
        Duration_of_pendulum_marker.nanosec = int(self.state_update_timeperiod * 1e+9)
        pendulum_marker.lifetime = Duration_of_pendulum_marker  
        self.visualizer.publish(pendulum_marker)

        self.obj_id += 1

    

def main(args = None):

    rclpy.init(args = args)
    pendulum_ = double_pendulum()
    rclpy.spin(pendulum_)
    pendulum_.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
