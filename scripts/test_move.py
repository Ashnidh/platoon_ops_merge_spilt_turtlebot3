#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math


BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1


class TestMoveNode(Node):
    def __init__(self):
        super().__init__("test_move")
        self.cmd_vel_pub = self.create_publisher(Twist, '/v_1/cmd_vel', 10)
        self.timer_ = self.create_timer(0.01, self.send_velocity_command)
        self.time_step = 0.0
        self.get_logger().info("Test Move Chaalu!!")
    
    def send_velocity_command(self):
        msg = Twist()
        msg.linear.x = 3.0  # Constant forward speed
        
        # Sine wave angular velocity
        msg.angular.z = math.sin(self.time_step) * 0.0  # Adjust multiplier for sharper turns
        
        # Clamp the angular velocity to avoid exceeding robot limits
        # msg.angular.z = clamp(msg.angular.z, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
        
        self.cmd_vel_pub.publish(msg)
        
        # Increment time stepx
        self.time_step += 0.002


def main(args=None):
    print("Andar")
    rclpy.init(args=args)
    node = TestMoveNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()