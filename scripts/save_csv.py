#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import csv
import os
from datetime import datetime

"""

This node saves the position, distance covered and control inputs for each bot as a CSV file.

"""

NUMBER_OF_BOTS = 4

class MultiCmdVelLogger(Node):
    def __init__(self):
        super().__init__('cmd_vel_logger')

        # List of topics to subscribe to
        self.robot_cmd_vel_topics = [f'/v_{i}/cmd_vel' for i in range(1, NUMBER_OF_BOTS+1)]
        self.robot_plot_topics = [f'/v_{i}/plot' for i in range(1, NUMBER_OF_BOTS+1)]
        
        # Dictionary to store file handles
        self.files = {}
        self.writers = {}

        # Subscription maps
        self.subscription_map = []

        # Subscribe to cmd_vel topics
        for topic in self.robot_cmd_vel_topics:
            sub = self.create_subscription(Twist, topic, self.create_cmd_vel_callback(topic), 10)
            self.subscription_map.append(sub)
            self.setup_csv(topic, is_plot=False)

        # Subscribe to plot topics
        for topic in self.robot_plot_topics:
            sub = self.create_subscription(Float32MultiArray, topic, self.create_plot_callback(topic), 10)
            self.subscription_map.append(sub)
            self.setup_csv(topic, is_plot=True)

    def setup_csv(self, topic, is_plot=False):
        """ Initialize CSV file for the given topic """
        file_name = f"{topic.replace('/', '_')}.csv"
        self.files[topic] = open(file_name, mode='w', newline='')
        self.writers[topic] = csv.writer(self.files[topic])

        # Write header
        if is_plot:
            self.writers[topic].writerow(["timestamp", "plot_x", "plot_y", "plot_Lh"])
        else:
            self.writers[topic].writerow(["timestamp", "linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z"])

        # self.get_logger().info(f"Logging data to {file_name}")

    def create_cmd_vel_callback(self, topic):
        """ Creates a callback function for cmd_vel topic """
        def callback(msg):
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
            data = [timestamp, msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z]
            self.writers[topic].writerow(data)
            self.files[topic].flush()  # Ensure data is written
            # self.get_logger().info(f"Logged data for {topic}: {data}")
        return callback

    def create_plot_callback(self, topic):
        """ Creates a callback function for plot topic """
        def callback(msg):
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
            data = [timestamp, msg.data[0], msg.data[1], msg.data[2]]  
            self.writers[topic].writerow(data)
            self.files[topic].flush()  # Ensure data is written
            # self.get_logger().info(f"Logged data for {topic}: {data}")
        return callback

    def destroy_node(self):
        """ Ensure files are closed properly """
        for file in self.files.values():
            file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MultiCmdVelLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
