import rclpy
from rclpy.node import Node
import csv
import math
import os
from datetime import datetime

# ROS 2 Standard Messages
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import Float64MultiArray 

class NNDataCollector(Node):
    def __init__(self):
        super().__init__('nn_data_collector')
        
        # --- 1. DYNAMIC PATH SETUP ---
        # This will save the CSV in the folder where you run the 'ros2 run' command
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f'nn_data_{timestamp_str}.csv'
        
        try:
            # Open file immediately
            self.csv_file = open(self.filename, 'w', newline='', buffering=1)
            self.csv_writer = csv.writer(self.csv_file)
            
            self.csv_writer.writerow([
                'timestamp', 
                'amcl_x', 'amcl_y', 'amcl_theta', 
                'dubins_x', 'dubins_y', 'dubins_v', 'dubins_theta',
                'teleop_v', 'teleop_w'
            ])
            self.get_logger().info(f"✅ FILE CREATED SUCCESSFULLY: {os.path.abspath(self.filename)}")
        except Exception as e:
            self.get_logger().error(f"❌ FAILED TO CREATE FILE: {e}")
            raise e

        # --- 2. Data Buffers ---
        self.latest_amcl = None
        self.latest_dubins = None
        self.latest_teleop = Twist()

        # --- 3. Standard Subscribers ---
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)
        self.create_subscription(Float64MultiArray, '/dubins_state', self.dubins_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.teleop_callback, 10)

        # --- 4. Sampling Timer (10Hz) ---
        self.timer = self.create_timer(0.1, self.save_data_to_csv)

        self.get_logger().info("📡 Waiting for /amcl_pose and /dubins_state to start recording...")
        self.count = 0

    def amcl_callback(self, msg):
        if self.latest_amcl is None:
            self.get_logger().info("📥 Received first AMCL message!")
        self.latest_amcl = msg

    def dubins_callback(self, msg):
        if self.latest_dubins is None:
            self.get_logger().info("📥 Received first Dubins message!")
        self.latest_dubins = msg

    def teleop_callback(self, msg):
        self.latest_teleop = msg

    def save_data_to_csv(self):
        # The file is created, but we don't write rows until we have data
        if self.latest_amcl is None:
            return
        if self.latest_dubins is None:
            return

        try:
            now = self.get_clock().now().to_msg()
            timestamp = now.sec + (now.nanosec * 1e-9)

            # Extract AMCL
            amcl_x = self.latest_amcl.pose.pose.position.x
            amcl_y = self.latest_amcl.pose.pose.position.y
            q = self.latest_amcl.pose.pose.orientation
            amcl_theta = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

            # Extract Dubins
            d_x, d_y, d_v, d_theta = self.latest_dubins.data[0:4]

            # Extract Teleop
            t_v = self.latest_teleop.linear.x
            t_w = self.latest_teleop.angular.z

            self.csv_writer.writerow([timestamp, amcl_x, amcl_y, amcl_theta, d_x, d_y, d_v, d_theta, t_v, t_w])
            
            self.count += 1
            if self.count % 50 == 0:
                self.get_logger().info(f"✍️ Recorded {self.count} samples...")
        except Exception as e:
            self.get_logger().error(f"Error during logging: {e}")

    def destroy_node(self):
        self.get_logger().info(f"Finalizing... Saved {self.count} samples.")
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