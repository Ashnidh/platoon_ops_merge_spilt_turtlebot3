#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import rclpy
from rclpy.node import Node
from collections import deque
import threading
import time
import math
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


"""

This node publishes command velocity to follower
Includes Obstacle avoidance algorithm  (Refer to paper in README file)
Also it dynamically switches leader reference trajectory based on platoon info 

"""

# Ref State Elements:{x_r,y_r,theta_r,v_ff,w_ff}

class FollowerMoveNode(Node):

    # robot_no = 2
    # leader_bot = 1
    # queue_update_counter = 0
    # L = 1.5
    L = 0.8
    # init_dist = 1.0
    # Lh = -1.0 * (robot_no - 1) * init_dist
    # Lh_init = -1.0 * (robot_no - 1) * init_dist
    
    def __init__(self, robot_no, leader_bot):
        super().__init__('follower_move')
        self.ref_subscription_ = None
        self.current_leader_ref_topic = None

        self.robot_no = robot_no
        self.leader_bot = leader_bot
        self.platoon_no = 1
        # Check current leader bot for each robot
        self.get_logger().info(f'robot_no: {self.robot_no}')
        self.get_logger().info(f'leader_bot: {self.leader_bot}')
        
        
        # Declare and get parameters
        # self.declare_parameter('queue_size', 100)
        # self.declare_parameter('ref_sub_frequency', 2.0)
        self.declare_parameter('update_frequency', 500.0)
        
        # queue_size = self.get_parameter('queue_size').value
        # self.ref_sub_frequency = self.get_parameter('ref_sub_frequency').value
        self.update_frequency = self.get_parameter('update_frequency').value

        # State Some Variables
        self.v_fb = 0.0
        self.W_fb = 0.0
        self.k1 = 1.0
        self.k2 = 2.0
        self.k3 = 0.8
        
        
        self.e = np.array([0.0, 0.0, 0.0])
        self.theta = 0.0
        self.curr_pose = (0.0, 0.0)
        # self.prev_pose = (0.0, 0.0)
        # self.curr_vel = (0.0, 0.0)
        # self.speed = 0.0
        self.X_r = np.array([0.0, 0.0])
        self.theta_r = 0.0
        self.v_ff = 0.0
        self.W_ff = 0.0
        self.beta = 0.0
        self.alpha = 0.0
        self.o_mag = math.inf
        self.d = math.inf
        self.B = (4 * self.L / 5)
        self.repulsive_vec = np.array([0.0, 0.0])
        self.update_vel_counter = 0
        self.integral = 0

        
        # Queue to store positions
        # self.position_queue = deque(maxlen=queue_size)
        

        self.cmd_vel_pub = self.create_publisher(Twist, f'v_{self.robot_no}/cmd_vel', 10)

        
        # Subscribe to reference coordinates
        
        self.ref_subscription_ = self.create_subscription(
            Float32MultiArray,
            f'v_{self.leader_bot}/ref',
            self.ref_callback,
            10  # QoS profile depth
        )
        # Get platoon info for each bot
        self.vars_subscription_ = self.create_subscription(
            Int16MultiArray,
            f'v_{self.robot_no}/platoon_info',
            self.vars_subscription_callback,
            10  # QoS profile depth
        )
        # Get current position of each robot
        self.pose_subscription_ = self.create_subscription(
            PoseArray,
            f'v_{self.robot_no}/pose',
            self.pose_callback,
            10  # QoS profile depth
        )
        # Subcribe to laser scan data for obstacle detection
        self.subscription = self.create_subscription(
            LaserScan,
            f'v_{self.robot_no}/scan',
            self.scan_callback,
            10
        )
        
        # self.reference_publisher_ = self.create_publisher(Float32MultiArray, f'v_{self.robot_no}/ref', 10)
        # self.reference_publisher_timer = self.create_timer(0.5, self.publish_reference)
        
        # Thread for velocity
        self.lock = threading.Lock()
        # self.timer_ref_sub = threading.Thread(target=self.ref_subscription_function, daemon=True)
        self.timer_velocity = threading.Thread(target=self.get_error, daemon=True)
        self.vel_pub_timer_ = self.create_timer(0.01, self.send_velocity_command)

        # Start threads
        # self.timer_ref_sub.start()
        self.timer_velocity.start()
        
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
     
        
    def ref_callback(self, msg):
        
        # Callback to process reference data
        
        with self.lock:
            if msg.data:
                self.X_r = np.array([(msg.data)[0], (msg.data)[1]])
                self.theta_r = (msg.data)[2]
                self.v_ff = (msg.data)[3]
                self.W_ff = (msg.data)[4]
                
                
                # self.get_logger().info(f'X_r: {self.X_r}')
                # self.get_logger().info(f'theta_r: {self.theta_r}')
                # self.get_logger().info(f'V_ff: {self.v_ff}')
                # self.get_logger().info(f'W_ff: {self.W_ff}')
        
    def vars_subscription_callback(self, msg):
        
        # Callback to process platoon info
        
        with self.lock:
            if msg.data:
                new_platoon = msg.data[0]
                new_leader = msg.data[1]
                new_topic = f'v_{new_leader}/ref'
                # Update leader reference topic dynamically
                if new_topic != self.current_leader_ref_topic:
                    self.get_logger().info(f"Switching leader from {self.current_leader_ref_topic} to {new_topic}")
                    
                    # Destroy old subscription
                    if self.ref_subscription_:
                        self.destroy_subscription(self.ref_subscription_)
                    
                    # Create new subscription to update leader reference trajectory
                    self.ref_subscription_ = self.create_subscription(
                        Float32MultiArray,
                        new_topic,
                        self.ref_callback,
                        10
                    )
                    self.current_leader_ref_topic = new_topic

                self.leader_bot = new_leader
                self.platoon_no = new_platoon

                
                # self.get_logger().info(f'robot_no: {self.robot_no} leader_no: {self.leader_bot}')
                # self.get_logger().info(f'robot_no: {self.robot_no} platoon_no: {self.platoon_no}')
                # self.get_logger().info(f'X_r: {self.X_r}')
                # self.get_logger().info(f'theta_r: {self.theta_r}')
                # self.get_logger().info(f'V_ff: {self.v_ff}')
                # self.get_logger().info(f'W_ff: {self.W_ff}')
        
    def pose_callback(self, msg):
        
        # Callback to process pose data
        
        with self.lock:
            if msg.poses:

                first_pose = msg.poses[0]
                self.curr_pose = (first_pose.position.x, first_pose.position.y)

                quaternion = first_pose.orientation
                self.theta = self.normalize_angle_rad(self.quaternion_to_theta(quaternion))

                # self.get_logger().info(f'curr_pose: {self.curr_pose}')
                # self.get_logger().info(f'theta: {self.theta}')
                
                # self.get_logger().info(f'Received Position: x={first_pose.position.x}, y={first_pose.position.y}')
                # self.get_logger().info(f'Received Theta: {self.theta}')
         
                
    def scan_callback(self, msg: LaserScan):
        # Convert ranges to numpy array
        ranges = np.array(msg.ranges)

        # Replace NaNs and infs
        cleaned_ranges = np.nan_to_num(ranges, nan=math.inf, posinf=math.inf)

        # Compute all angles corresponding to the scan
        angles = msg.angle_min + np.arange(len(cleaned_ranges)) * msg.angle_increment

        # Normalize all angles to (-pi, pi]
        normalized_angles = np.array([self.normalize_angle_rad(a) for a in angles])

        # Define angular window: from -60° to +60° => [-π/3, π/3]
        lower_bound = -math.pi / 4
        upper_bound = math.pi / 4

        # Mask: indices where angle is within the desired window
        mask = (normalized_angles >= lower_bound) & (normalized_angles <= upper_bound)

        # Apply mask
        filtered_ranges = cleaned_ranges[mask]
        filtered_angles = normalized_angles[mask]

        if len(filtered_ranges) == 0:
            self.get_logger().warn("No valid points in front field of view.")
            return

        # Find closest point in filtered data
        min_idx = np.argmin(filtered_ranges)
        min_distance = filtered_ranges[min_idx]
        closest_angle = filtered_angles[min_idx]

        # Save results
        self.o_mag = min_distance
        self.beta = self.normalize_angle_rad(closest_angle)
        
        # Logging
        # self.get_logger().info(
        #     f'Robot_No: {self.robot_no}, Closest point in front: {min_distance:.2f} meters at angle {math.degrees(self.beta):.2f}°'
        # )
        
    
    def get_error(self):
        
        # Timer thread to calculate error
        
        rate = 1.0 / self.update_frequency
        while rclpy.ok():
            with self.lock:
                transformation_matrix = np.array([
                    [math.cos(self.theta), math.sin(self.theta), 0.0],
                    [-math.sin(self.theta), math.cos(self.theta), 0.0],
                    [0.0, 0.0, 1.0]
                ])
                
                diff_mat = np.array([
                    (self.X_r)[0] - (self.curr_pose)[0], 
                    (self.X_r)[1] - (self.curr_pose)[1], 
                    self.theta_r - self.theta
                ]).reshape(3, 1)
                
                self.e = np.dot(transformation_matrix, diff_mat)
                

                self.alpha = self.normalize_angle_rad(math.atan((self.e)[1] / (self.e)[0]))
                

                
                n = self.get_n()

                if self.d < self.B:
                    (self.e)[0] = 2*n[0]
                    (self.e)[1] = 2*n[1]
                    (self.e)[2] = self.normalize_angle_rad(math.atan(2*n[1]/n[0]))

                self.alpha = math.atan((self.e)[1] / (self.e)[0])

                self.getV_fb()
                self.getW_fb()
                
                
                
            # self.get_logger().info(
            #     f'self.alpha: {math.degrees(self.alpha)}°'
            # )             

                # self.get_logger().info(f'X_r - X: {diff_mat}')

                
            time.sleep(rate)  # Move sleep outside lock

    def getV_fb(self):
        self.v_fb = self.k1 * (self.e)[0]  

    def getW_fb(self):
        self.W_fb = self.k2 * (self.e)[1]    + self.k3 * (self.e)[2]  
        
    def send_velocity_command(self):
        msg = Twist()
        if self.platoon_no ==2 and self.robot_no == 2:
            self.update_vel_counter += 1
            if self.update_vel_counter < 480:
                msg.linear.x = 3.0
                msg.angular.z = -3.0
            elif self.update_vel_counter < 965:
                msg.linear.x = 3.0
                msg.angular.z = 3.0
            else:
                msg.linear.x = 4.0
                msg.angular.z = 0.0


        # if self.platoon_no ==3 and self.robot_no == 4:
        #     msg.linear.x = 3.0
        #     msg.angular.z = 0.2
            
        else:
            if self.v_ff == 0.0:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
            else:
                
                msg.linear.x = float(self.v_ff * math.cos(float(self.e[2].item()))) + float(self.v_fb)

                # print(f"self.v_ff type: {type(self.v_ff)}, value: {self.v_ff}")
                # print(f"self.e[2] type: {type(self.e[2])}, value: {self.e[2]}")
                # print(f"self.e[2].item() type: {type(self.e[2].item())}, value: {self.e[2].item()}")
                # print(f"self.v_fb type: {type(self.v_fb)}, value: {self.v_fb}")
                
                msg.angular.z = (float)(self.W_ff + self.W_fb)  # Adjust multiplier for sharper turns
            
                # Clamp the angular velocity to avoid exceeding robot limits
                # msg.angular.z = clamp(msg.angular.z, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
            

        self.cmd_vel_pub.publish(msg)


    def get_n (self):
        ox = self.o_mag * math.cos(self.beta)
        oy = self.o_mag * math.sin(self.beta)
        # ox = self.o_mag * math.cos(self.beta)
        # oy = self.o_mag * math.sin(self.beta)
    
        o = np.array([ox, oy])
        
        self.d = np.abs(oy * math.cos(self.alpha) - ox * math.sin(self.alpha))
        
        a_mag = math.sqrt((self.e)[0] ** 2 + (self.e)[1] ** 2)
        a_angle = self.alpha
        
        a = np.array([(a_mag * math.cos(a_angle)), (a_mag * math.sin(a_angle))])
        
        g = np.dot(a, o) / (np.linalg.norm(a) ** 2)
        
        n = a
        if self.d < self.B:
            n = a + a - (o/g)

        # self.get_logger().info(
        #     f'Robot_No: {self.robot_no}, DDDDDDDDDDDDDDDDDDDDDDDDDDD: {self.d:.2f}'
        # )
        # self.get_logger().info(
        #     f'Robot_No: {self.robot_no}, TTTHHHEETTAAA: {math.degrees(self.theta):.2f}'
        # )
        # self.get_logger().info(
        #     f'Robot_No: {self.robot_no}, ALLPPHAA: {math.degrees(self.alpha):.2f}'
        # )
        # self.get_logger().info(
        #     f'Robot_No: {self.robot_no}, BEEETTTAA: {math.degrees(self.beta):.2f}'
        # )
        # self.get_logger().info(
        #     f'Robot_No: {self.robot_no}, OOXXXXX: {ox:.2f}'
        # )
        # self.get_logger().info(
        #     f'Robot_No: {self.robot_no}, OOYYYYY: {oy:.2f}'
        # )

        
        return n


def main(args=None):
    print("Inside main")
    parser = argparse.ArgumentParser(description='Started')
    parser.add_argument('-l', '--leader_bot', type=str, default='1', help='Name of the robot to spawn')
    parser.add_argument('-n', '--robot_no', type=str, default='1', help='Number of robot acting upon')
    
    args, unknown = parser.parse_known_args()
    # config = vars(args)

    # args.max_vel = float(args.max_vel)
    args.robot_no = (int)(args.robot_no)
    args.leader_bot = (int)(args.leader_bot)
    
    try:
        rclpy.init(args=unknown)
        node = FollowerMoveNode(args.robot_no, args.leader_bot)
        rclpy.spin(node)
    finally:
        print("___________________FollowMove___________")
        rclpy.shutdown()

if __name__ == '__main__':
    main()