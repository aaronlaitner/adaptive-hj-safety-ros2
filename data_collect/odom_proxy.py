import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import math

class OdomToDubins(Node):
    def __init__(self):
        super().__init__('odom_to_dubins_proxy')
        self.sub = self.create_subscription(Odometry, '/odom', self.callback, 10)
        self.pub = self.create_publisher(Float64MultiArray, '/dubins_state', 10)

    def callback(self, msg):
        # Convert Quaternion to Yaw (theta)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Publish in the format your collector expects: [x, y, v, theta]
        proxy_msg = Float64MultiArray()
        proxy_msg.data = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.twist.twist.linear.x,
            yaw
        ]
        self.pub.publish(proxy_msg)

def main():
    rclpy.init()
    rclpy.spin(OdomToDubins())
    rclpy.shutdown()