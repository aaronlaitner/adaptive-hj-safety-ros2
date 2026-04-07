import rclpy
from rclpy.node import Node
import message_filters
import csv
import math
import os
from datetime import datetime

# ROS 2 Standard Messages
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist  # <-- Added for Teleop commands
from std_msgs.msg import Float64MultiArray 

class NNDataCollector(Node):
    def __init__(self):
        super().__init__('nn_data_collector')
        
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f'nn_training_data_{timestamp_str}.csv'
        self.csv_file = open(self.filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        self.csv_writer.writerow([
            'timestamp', 
            'amcl_x', 'amcl_y', 'amcl_theta', 
            'dubins_x', 'dubins_y', 'dubins_v', 'dubins_theta',
            'teleop_v', 'teleop_w'  # <-- New Teleop Headers
        ])

        # 1. AMCL Pose Subscriber
        self.amcl_sub = message_filters.Subscriber(
            self, 
            PoseWithCovarianceStamped, 
            '/amcl_pose'
        )
            
        # 2. Dubins State Subscriber
        self.dubins_sub = message_filters.Subscriber(
            self, 
            Float64MultiArray, 
            '/dubins_state'
        )

        # 3. Teleop Command Subscriber (Usually published by teleop_twist_joy or teleop_twist_keyboard)
        self.teleop_sub = message_filters.Subscriber(
            self,
            Twist,
            '/cmd_vel'  
        )

        # 4. Approximate Time Synchronizer (Now with 3 topics)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.amcl_sub, self.dubins_sub, self.teleop_sub], # <-- Added teleop_sub
            queue_size=100, 
            slop=0.1,
            # allow_headerless=True is required because Twist and Float64MultiArray 
            # don't have built-in timestamp headers; sync relies on arrival time.
            allow_headerless=True 
        )
        
        self.ts.registerCallback(self.sync_callback)
        self.get_logger().info(f"Data Collector initialized. Writing to {self.filename}...")
        self.get_logger().info("Waiting for synchronized messages on /amcl_pose, /dubins_state, and /cmd_vel...")

    def sync_callback(self, amcl_msg, dubins_msg, teleop_msg): # <-- Added teleop_msg arg
        """
        Triggered only when messages from all THREE topics arrive within the 'slop' time window.
        """
        # --- Extract Time ---
        timestamp = amcl_msg.header.stamp.sec + (amcl_msg.header.stamp.nanosec * 1e-9)

        # --- Extract AMCL Data ---
        amcl_x = amcl_msg.pose.pose.position.x
        amcl_y = amcl_msg.pose.pose.position.y
        
        q = amcl_msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        amcl_theta = math.atan2(siny_cosp, cosy_cosp)

        # --- Extract Dubins Data ---
        try:
            dubins_x = dubins_msg.data[0]
            dubins_y = dubins_msg.data[1]
            dubins_v = dubins_msg.data[2]
            dubins_theta = dubins_msg.data[3]
        except IndexError:
            self.get_logger().error("Dubins state array does not contain 4 elements! Check publisher.")
            return

        # --- Extract Teleop Data ---
        # Twist messages contain linear (x,y,z) and angular (x,y,z) velocities.
        # For a 2D Dubins car, you only care about forward speed (linear.x) and steering/yaw (angular.z)
        teleop_v = teleop_msg.linear.x
        teleop_w = teleop_msg.angular.z

        # --- Write to CSV ---
        self.csv_writer.writerow([
            timestamp, 
            amcl_x, amcl_y, amcl_theta, 
            dubins_x, dubins_y, dubins_v, dubins_theta,
            teleop_v, teleop_w  # <-- Added to CSV output
        ])

    def destroy_node(self):
        self.get_logger().info("Shutting down... saving CSV.")
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NNDataCollector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()