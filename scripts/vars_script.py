#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import argparse
import rclpy
from rclpy.node import Node
import math
import numpy as np
import time
import threading
from std_msgs.msg import Int16MultiArray


class VarMaintainNode(Node):

    
    def __init__(self, robot_no, platoon_no):
        super().__init__('var_maintain_node')
        
        self.robot_no = robot_no
        self.platoon_no = platoon_no
        self.leader_bot = self.robot_no - 1
        self.leader_set = True

        # Declare and get parameters
        self.declare_parameter('leader_update_frequency', 100)
        # self.declare_parameter('queue_size', 100)
        # self.declare_parameter('queue_update_frequency', 1.0)
        # self.declare_parameter('velocity_update_frequency', 700.0)
        
        self.leader_update_frequency = self.get_parameter('leader_update_frequency').value
        # queue_size = self.get_parameter('queue_size').value
        # self.queue_update_frequency = self.get_parameter('queue_update_frequency').value
        # self.velocity_update_frequency = self.get_parameter('velocity_update_frequency').value

        # State Some Variables
        self.theta = 0.0
        self.curr_pose = (0.0, 0.0)
        self.prev_pose = (0.0, 0.0)
        self.curr_vel = (0.0, 0.0)
        self.speed = 0.0
        self.X_r = np.array([0.0, 0.0])
        self.theta_r = 0.0
        self.v_ff = 0.0
        self.W_ff = 0.0
        
        # Queue to store positions
        # self.position_queue = deque(maxlen=queue_size)
        
        # Create a subscriber
        # self.leader_subscription_ = self.create_subscription(
        #     Int16MultiArray,
        #     f'/v_{self.leader_bot}/platoon_info',
        #     self.leader_subscription_callback,
        #     10  # QoS profile depth
        # )
        
        
        self.latest_platoon_data = {}
        
        # === Dynamic topic subscription ===
        self.other_robots = [i for i in range(1, robot_no)]  # update with relevant robot IDs
        self.subscribers = []

        for i in self.other_robots:
            if i != self.robot_no:  # Skip self
                topic = f'/v_{i}/platoon_info'
                subscriber = self.create_subscription(
                    Int16MultiArray,
                    topic,
                    self.generate_callback(i),
                    10
                )
                self.subscribers.append(subscriber)
        

        self.vars_subscription_ = self.create_subscription(
            Int16MultiArray,
            f'v_{self.robot_no}/platoon_info',
            self.vars_subscription_callback,
            100  # QoS profile depth
        )
        

        self.platoon_info_publisher_ = self.create_publisher(Int16MultiArray, f'/v_{self.robot_no}/platoon_info', 10)
        self.platoon_info_publisher_timer = self.create_timer(0.01, self.publish_platoon_info)
        
        # self.plot_publisher_ = self.create_publisher(Float32MultiArray, f'/v_{self.robot_no}/plot', 10)
        # self.plot_publisher_timer = self.create_timer(0.01, self.publish_plot)
        
        # Thread for velocity
        # self.lock = threading.Lock()
        # self.timer_check_leader = threading.Thread(target=self.update_leader, daemon=True)
        # self.timer_Position_queue = threading.Thread(target=self.run_timer, daemon=True)
        # self.timer_velocity = threading.Thread(target=self.get_vel, daemon=True)

        # Start threads
        # self.timer_check_leader.start()
        # self.timer_Position_queue.start()
        # self.timer_velocity.start()
        
    
    def generate_callback(self, robot_id):
        def callback(msg):
            if len(msg.data) > 1:
                sender_platoon = msg.data[0]
                sender_leader = msg.data[1]

                # Store the most recent message for this robot
                self.latest_platoon_data[robot_id] = msg.data

                # If this is the current leader but has a different platoon, leader might need updating
                if robot_id == self.leader_bot and sender_platoon != self.platoon_no:
                    self.leader_set = False
                    self.get_logger().info(f"Leader {robot_id} not in our platoon ({sender_platoon} != {self.platoon_no}), updating...")
                    self.find_new_leader()

        return callback
    
    def find_new_leader(self):
        for rid in range(self.robot_no - 1, 0, -1):  # From robot_no-1 down to 1
            if rid in self.latest_platoon_data:
                platoon_no = self.latest_platoon_data[rid][0]
                if platoon_no == self.platoon_no:
                    self.leader_bot = rid
                    self.leader_set = True
                    self.get_logger().info(f"New leader found: Robot {rid}")
                    return

        # If no leader found
        self.get_logger().warn("No valid leader found in the same platoon.")


    def vars_subscription_callback(self, msg):
        
        # Callback to process reference data
    
        if msg.data:
            self.platoon_no = msg.data[0]
            self.leader_bot = msg.data[1]
                

    def publish_platoon_info(self):
        msg = Int16MultiArray()
        msg.data = [self.platoon_no, self.leader_bot]
        # Check if data is valid before publishing
        if all(not math.isnan(val) for val in msg.data):  
            self.platoon_info_publisher_.publish(msg)
            # self.get_logger().info(f"Published Reference: {msg.data}")
        else:
            self.get_logger().warn("Reference contains NaN values. Skipping publish.")

def main(args=None):
    print("Andar")
    parser = argparse.ArgumentParser(description='Hehe Starting mein toh yahi chalega!')
    # parser.add_argument('-v', '--max_vel', type=str, default='1.0', help='Name of the robot to spawn')
    parser.add_argument('-n', '--robot_no', type=str, default='1', help='Number of robot acting upon')
    parser.add_argument('-p', '--platoon_no', type=str, default='1', help='Platoon of the robot concerned')
    
    args, unknown = parser.parse_known_args()
    # config = vars(args)

    # args.max_vel = float(args.max_vel)
    args.robot_no = (int)(args.robot_no)
    args.platoon_no = (int)(args.platoon_no)

    try:
        rclpy.init(args=unknown)
        node = VarMaintainNode(args.robot_no, args.platoon_no)
        rclpy.spin(node)
    finally:
        print("________________Var_Maintain___________")
        rclpy.shutdown()

if __name__ == '__main__':
    main()