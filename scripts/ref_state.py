#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from collections import deque
import threading
import time
import math
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool 


"""

This node generates and publishes reference coordinates for each robot.Subscribes to merge trigger, current position of robot.
Refer to paper mentioned in README for equations used.

"""

""" Queue Elements:{Lh,x,y,vx,vy,theta,timestamp} """


class ReferenceStateNode(Node):

    # robot_no = 1
    queue_update_counter = 0
    L = 0.8             # safety distance between bots
    
    def __init__(self, robot_no):
        super().__init__('ref_state_node')
        
        # Subscriber for merge operation to incresae distance
        self.trigger_subscription = self.create_subscription(
            Bool,
            'trigger_move',
            self.trigger_callback,
            10
        )
        self.trigger_subscription
        
        self.L_increase = False       # increment safety distance L only during merging
        self.robot_no = robot_no

        self.Lh = 0.0
        
        # Declare and get parameters
        self.declare_parameter('queue_size', 100)
        self.declare_parameter('queue_update_frequency', 1.0)
        self.declare_parameter('velocity_update_frequency', 700.0)
        
        queue_size = self.get_parameter('queue_size').value
        self.queue_update_frequency = self.get_parameter('queue_update_frequency').value
        self.velocity_update_frequency = self.get_parameter('velocity_update_frequency').value

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
        self.position_queue = deque(maxlen=queue_size)
        
        # Subscribe to get current postion
        self.subscription = self.create_subscription(
            PoseArray,
            f'/v_{self.robot_no}/pose',
            self.pose_callback,
            10  # QoS profile depth
        )
        # Publish reference variables
        self.reference_publisher_ = self.create_publisher(Float32MultiArray, f'/v_{self.robot_no}/ref', 10)
        self.reference_publisher_timer = self.create_timer(0.01, self.publish_reference)
        
        # Publish current pose and distance for plotting
        self.plot_publisher_ = self.create_publisher(Float32MultiArray, f'/v_{self.robot_no}/plot', 10)
        self.plot_publisher_timer = self.create_timer(0.01, self.publish_plot)
        
        # Thread for velocity
        self.lock = threading.Lock()
        self.timer_Position_queue = threading.Thread(target=self.run_timer, daemon=True)
        self.timer_velocity = threading.Thread(target=self.get_vel, daemon=True)

        # Start threads
        self.timer_Position_queue.start()
        self.timer_velocity.start()
        
        
    def trigger_callback(self, msg: Bool):
        if msg.data:
            self.L_increase = True
        
        
    def pose_callback(self, msg):
        
        # Callback to process pose data
        
        with self.lock:
            if msg.poses:

                first_pose = msg.poses[0]
                self.curr_pose = (first_pose.position.x, first_pose.position.y)

                quaternion = first_pose.orientation
                self.theta = self.quaternion_to_theta(quaternion)
                
                # self.get_logger().info(f'Received Position: x={first_pose.position.x}, y={first_pose.position.y}')
                # self.get_logger().info(f'Received Theta: {self.theta}')
                
                

    def run_timer(self):
        # Timer thread to Update the queue
        rate = 1.0 / self.queue_update_frequency
        while rclpy.ok():
            with self.lock:
                self.queue_update_counter += 1
                self.position_queue.append((self.Lh, *self.curr_pose, *self.curr_vel, self.theta, self.queue_update_counter*rate))
                if self.L_increase and self.robot_no == 1 and self.L <= 2:
                    self.L += 0.1  # increase L for merge trigger
                # self.get_logger().info(f'Queue Content: {list(self.position_queue)}')
                # self.get_logger().info(f'Velocity: {math.sqrt(self.curr_vel[0] ** 2 + self.curr_vel[1] ** 2)}')
                # self.get_logger().info(f'Lh for robot {self.robot_no}: {self.Lh}')
                # self.get_logger().info(f'Speed: {self.speed}')

                self.search_position_queue()

            time.sleep(rate)
                

    def get_vel(self):
        
        # Timer thread to calculate velocity
        
        rate = 1.0 / self.velocity_update_frequency
        while rclpy.ok():
            with self.lock:
                if self.prev_pose != (0.0, 0.0):  # Ensure prev_pose is valid
                    vx = (self.curr_pose[0] - self.prev_pose[0]) / rate
                    vy = (self.curr_pose[1] - self.prev_pose[1]) / rate
                    self.curr_vel = (vx, vy)                  
                    self.speed = math.sqrt(vx ** 2 + vy ** 2)
                    self.Lh += self.speed * rate  # Update Lh based on speed

                # Update prev_pose *after* computing velocity
                self.prev_pose = self.curr_pose  

            time.sleep(rate)  # Move sleep outside lock
                    
            
    def normalize_angle_rad(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi
 
    def quaternion_to_theta(self, quaternion):
        
        qw = quaternion.w
        qx = quaternion.x
        qy = quaternion.y
        qz = quaternion.z
        
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        theta = math.atan2(siny_cosp, cosy_cosp)

        return self.normalize_angle_rad(theta)

    # Using Distance function to calculate reference coordinates.
    def search_position_queue(self):
        LhT = max(self.Lh - self.L, 0)
        if(self.Lh - self.L > 0):
            considered_tuples = []
            
            closest_tuple = min(self.position_queue, key = lambda x: abs(x[0] - LhT))
            # self.get_logger().info(f"Closest First Element of {LhT}: {closest_tuple}")

            index = next(i for i, tup in enumerate(self.position_queue) if tup == closest_tuple)
            next_tuples = [(self.position_queue)[i] for i in range(index+1, min(index+3, len(self.position_queue)))]

            considered_tuples.append(closest_tuple)
            considered_tuples.extend(next_tuples)
            
            self.T = closest_tuple[-1]
            
            # self.get_logger().info(f"Considered Tuples: {considered_tuples}")
            self.getCurveFittingCoeff(considered_tuples)

                
    def getCurveFittingCoeff(self, considered_tuples):
        V = np.array([[1, tup[-1], (tup[-1] ** 2) ] for tup in considered_tuples])
        # self.get_logger().info(f"V: {V}")
        Y_x = np.array([tup[1] for tup in considered_tuples])
        Y_y = np.array([tup[2] for tup in considered_tuples])

        V_pseudo_inv = np.linalg.pinv(V)
        self.a_x = np.dot(V_pseudo_inv, Y_x)
        self.a_y = np.dot(V_pseudo_inv, Y_y)
        
        # self.get_logger().info(f"V: {V}")
        # self.get_logger().info(f"Y_x: {Y_x}")
        # self.get_logger().info(f"Y_y: {Y_y}")
        # self.get_logger().info(f"a_x: {self.a_x}")
        # self.get_logger().info(f"a_y: {self.a_y}")

        self.getRefPosition()
        self.getRefTheta()
        self.getVff()
        self.getWff()
    
    def getRefPosition(self):
        x_h = (self.a_x)[2] * (self.T ** 2) + (self.a_x)[1] * self.T + (self.a_x)[0]
        y_h = (self.a_y)[2] * (self.T ** 2) + (self.a_y)[1] * self.T + (self.a_y)[0]
        self.X_r = np.array([x_h, y_h])
        # self.get_logger().info(f"X_r: {self.X_r}")        
    
    def getRefTheta(self):
        self.theta_r = self.normalize_angle_rad(math.atan((2 * (self.a_y)[2] * self.T + (self.a_y)[1]) / (2 * (self.a_x)[2] * self.T + (self.a_x)[1])))
        # self.get_logger().info(f"Theta_r: {self.theta_r}")
        
    def getVff(self):
        self.v_ff = math.sqrt((2 * (self.a_x)[2] * self.T + (self.a_x)[1]) ** 2 + (2 * (self.a_y)[2] * self.T + (self.a_y)[1]) ** 2)
        # self.get_logger().info(f"V_ff: {self.v_ff}")

    def getWff(self):
        self.W_ff = ((2 * (self.a_x)[2] * self.T + (self.a_x)[1]) * 2 * (self.a_y)[2] - (2 * (self.a_y)[2] * self.T + (self.a_y)[1]) * 2 * (self.a_x)[2]) / ((2 * (self.a_x)[2] * self.T + (self.a_x)[1]) ** 2 + (2 * (self.a_y)[2] * self.T + (self.a_y)[1]) ** 2)
        # self.get_logger().info(f"W_ff: {self.W_ff}")

    def publish_reference(self):
        msg = Float32MultiArray()
        msg.data = [*(self.X_r), self.theta_r, self.v_ff, self.W_ff]
        # Check if data is valid before publishing
        if all(not math.isnan(val) for val in msg.data):  
            self.reference_publisher_.publish(msg)
            # self.get_logger().info(f"Published Reference: {msg.data}")
        else:
            self.get_logger().warn("Reference contains NaN values. Skipping publish.")

    def publish_plot(self):
        msg = Float32MultiArray()
        msg.data = [*(self.curr_pose), self.Lh]
        # Check if data is valid before publishing
        if all(not math.isnan(val) for val in msg.data):  
            self.plot_publisher_.publish(msg)
            # self.get_logger().info(f"Published Reference: {msg.data}")
        else:
            self.get_logger().warn("Reference contains NaN values. Skipping publish.")


    


def main(args=None):
    print("Inside main")
    parser = argparse.ArgumentParser(description='Started')
    # parser.add_argument('-v', '--max_vel', type=str, default='1', help='Name of the robot to spawn')
    parser.add_argument('-n', '--robot_no', type=str, default='1', help='Number of robot acting upon')
    
    args, unknown = parser.parse_known_args()
    # config = vars(args)

    # args.max_vel = float(args.max_vel)
    args.robot_no = (int)(args.robot_no)

    try:
        rclpy.init(args=unknown)
        node = ReferenceStateNode(args.robot_no)
        rclpy.spin(node)
    finally:
        print("___________________PoseQueue___________")
        rclpy.shutdown()

if __name__ == '__main__':
    main()