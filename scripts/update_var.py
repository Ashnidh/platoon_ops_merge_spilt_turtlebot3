#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray
from copy import deepcopy

"""

This node is used for dynamic platoon assignment.It checks the current platoon info(platoon-1,platoon-2) and updates continously
Trigger this node to change the platoon info of set of predefined bots.


"""

class TestMoveNode(Node):
    def __init__(self):
        super().__init__("test_move")
        self.splitting_bots = [2, 4, 5]     # List of predefined bots to send to the platoon-2
        
        # Cache of latest received data per bot
        self.latest_msgs = {
            bot_no: [0, bot_no - 1]  # fallback default before any messages arrive
            for bot_no in self.splitting_bots
        }

        # Subscribe to current platoon info, publish updated platoon info for each bot.
        self.var_update_pubs = {}
        self.subs = {}

        for bot_no in self.splitting_bots:
            topic_name = f'/v_{bot_no}/platoon_info'
            self.var_update_pubs[bot_no] = self.create_publisher(Int16MultiArray, topic_name, 10)
            self.subs[bot_no] = self.create_subscription(
                Int16MultiArray,
                topic_name,
                lambda msg, b=bot_no: self.callback_platoon_info(msg, b),
                10
            )

        self.var_update_timer_ = self.create_timer(0.01, self.var_update_command)
        self.time_step = 0.0
        self.get_logger().info("Test Move node has started")
    
    def callback_platoon_info(self, msg, bot_no):
        # Update only if data has expected format
        if len(msg.data) >= 2:
            self.latest_msgs[bot_no] = msg.data
        else:
            self.get_logger().warn(f"Received malformed data from /v_{bot_no}/platoon_info")

    def var_update_command(self):
        for bot_no, publisher in self.var_update_pubs.items():
            # Deepcopy to avoid modifying cached list directly
            updated_data = deepcopy(self.latest_msgs[bot_no])
            updated_data[0] = 2  # Update only index 0

            msg = Int16MultiArray()
            msg.data = updated_data
            publisher.publish(msg)
        

def main(args=None):
    print("Inside main")
    rclpy.init(args=args)
    node = TestMoveNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()