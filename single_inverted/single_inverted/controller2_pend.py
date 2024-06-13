#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_msgs2.msg import States
from custom_msgs2.msg import TorqueInput
import numpy as np

class Controller_Pendulum(Node):
    def __init__(self):
        super().__init__("controller_pend")
        self.send_torque_cmd = self.create_publisher(TorqueInput,'/torque_input',10)
        self.createsubs = self.create_subscription(States,'/state_feedback',self.send_torque,10)
        self.kp = 10
        self.kd = 2
        self.ki = 0.3
        self.theta_ir = [0,0,0,0,0,0,0,0,0,0]
        self.dtheta = [0]

    def send_torque(self,msg:States):
        cmd = TorqueInput()
        if msg.theta > 0:
            theta = np.pi - msg.theta
        elif msg.theta < 0:
            theta = -np.pi - msg.theta
        else:
            theta = np.pi
        self.theta_ir.insert(0,theta)
        self.theta_ir.pop()
        self.dtheta.append(theta)
        self.dtheta1 = (self.dtheta[-1]-self.dtheta[-2])/0.01
        # till it reach upper half
        if -np.pi/2 < msg.theta < np.pi/2:
            if msg.theta_dot >= 0:
                cmd.torque_value = 5.0
            else:
                cmd.torque_value = -5.0
        else:
            cmd.torque_value = self.kp*theta+self.kd*self.dtheta1+self.ki*sum(self.theta_ir)
        self.send_torque_cmd.publish(cmd)

def main(args = None):
    rclpy.init(args=args)
    node = Controller_Pendulum()
    rclpy.spin(node)
    rclpy.shutdown()