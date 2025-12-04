import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math
import numpy as np

class SlamExplorer(Node):
    def __init__(self):
        super().__init__('slam_explorer')
        
        # 1. SETUP
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 2. MAPPING CONFIGURATION (Crucial for SLAM)
        self.MAX_SPEED = 0.15        # Very slow for good map quality
        self.TURN_SPEED = 0.4        # Slow turning to prevent map tearing
        self.WALL_DIST = 0.6         # Keep a healthy distance from walls
        self.STOP_DIST = 0.35        # Emergency stop

        self.lidar_data = None
        self.lidar_info = {'min': 0.0, 'inc': 0.0}

        self.create_timer(0.1, self.control_loop)

    def lidar_callback(self, msg):
        # Clean data immediately
        raw = np.array(msg.ranges)
        self.lidar_data = np.nan_to_num(raw, nan=0.0, posinf=5.0, neginf=0.0)
        self.lidar_info['min'] = msg.angle_min
        self.lidar_info['inc'] = msg.angle_increment

    def get_slice(self, center_deg, width_deg):
        if self.lidar_data is None: return 5.0
        
        # Calculate indices
        start_rad = math.radians(center_deg - width_deg/2)
        end_rad = math.radians(center_deg + width_deg/2)
        start_i = int((start_rad - self.lidar_info['min']) / self.lidar_info['inc'])
        end_i = int((end_rad - self.lidar_info['min']) / self.lidar_info['inc'])
        
        # Clamp and slice
        start_i = max(0, start_i)
        end_i = min(len(self.lidar_data), end_i)
        if start_i >= end_i: return 5.0
        
        chunk = self.lidar_data[start_i:end_i]
        valid = chunk[chunk > 0.05]
        return np.min(valid) if len(valid) > 0 else 5.0

    def control_loop(self):
        if self.lidar_data is None: return
        twist = Twist()

        # Check directions (Simulated "Whiskers")
        d_front = self.get_slice(0, 40)
        d_left = self.get_slice(45, 30)
        d_right = self.get_slice(-45, 30)

        # --- LOGIC 1: EMERGENCY STOP ---
        if d_front < self.STOP_DIST:
            self.get_logger().warn("Too close! Backing up slowly...")
            twist.linear.x = -0.05
            twist.angular.z = 0.0

        # --- LOGIC 2: WALL AVOIDANCE (Smooth Turn) ---
        elif d_front < self.WALL_DIST:
            # Wall ahead. Turn smoothly away from the closer side.
            twist.linear.x = 0.05 # Keep moving slightly (stops map jitter)
            
            if d_left > d_right:
                # Left is open, turn Left (positive)
                twist.angular.z = self.TURN_SPEED 
            else:
                # Right is open, turn Right (negative)
                twist.angular.z = -self.TURN_SPEED

        # --- LOGIC 3: OPEN SPACE CRUISING ---
        else:
            twist.linear.x = self.MAX_SPEED
            
            # Gentle wandering: biased towards the most open side
            # This prevents it from driving perfectly straight into a dead end
            if d_left > d_right + 0.5:
                twist.angular.z = 0.2 # Drift Left
            elif d_right > d_left + 0.5:
                twist.angular.z = -0.2 # Drift Right
            else:
                twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SlamExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()