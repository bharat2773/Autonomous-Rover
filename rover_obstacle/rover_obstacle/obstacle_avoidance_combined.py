import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math
import numpy as np

class F1TenthNavigator(Node):
    def __init__(self):
        super().__init__('f1tenth_navigator')
        
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # --- CONFIGURATION ---
        self.MAX_SPEED = 0.35
        self.BUBBLE_RADIUS = 0.50 # Size of safety bubble (meters)
        self.GAP_THRESHOLD = 1.0  # Meters (Depth to be considered a 'Gap')
        self.SAFETY_DIST = 0.40   # Emergency Stop Distance
        
        # SENSORS
        self.lidar_data = None
        self.us_readings = {
            'front': 5.0, 'front_left': 5.0, 'front_right': 5.0,
            'left': 5.0, 'right': 5.0, 'rear': 5.0
        }
        
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos)
        
        # Subscribe to Ultrasonics
        keys = list(self.us_readings.keys())
        for k in keys:
            self.create_subscription(LaserScan, f'/us_sensor/{k}', lambda m, key=k: self.us_cb(key, m), qos)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.05, self.control_loop) # 20Hz

        # STATE
        self.reversing = False
        self.reverse_timer = 0

    def us_cb(self, key, msg):
        valid = [r for r in msg.ranges if r > 0.02 and not math.isinf(r)]
        self.us_readings[key] = min(valid) if valid else 5.0

    def lidar_callback(self, msg):
        # Store raw data
        self.lidar_data = msg

    def preprocess_lidar(self, ranges, angle_min, angle_inc):
        """
        1. Clean Data
        2. Fuse Ultrasonics into the array (Virtual Obstacles)
        3. Create Safety Bubble
        """
        proc_ranges = np.array(ranges)
        proc_ranges = np.nan_to_num(proc_ranges, nan=0.0, posinf=5.0, neginf=0.0)
        
        # --- FUSION: OVERWRITE LIDAR WITH ULTRASONICS ---
        # Map Ultrasonic angles to Lidar Indices
        # Front Ultrasonic covers center +/- 15 deg
        def fuse_us(key, center_deg, width_deg):
            dist = self.us_readings[key]
            if dist < 2.0: # Only fuse if obstacle is relevant
                idx_center = int((math.radians(center_deg) - angle_min) / angle_inc)
                idx_width = int(math.radians(width_deg) / angle_inc)
                
                start = max(0, idx_center - idx_width//2)
                end = min(len(proc_ranges), idx_center + idx_width//2)
                
                # If US is closer than Lidar, overwrite it
                if start < end:
                    proc_ranges[start:end] = np.minimum(proc_ranges[start:end], dist)

        fuse_us('front', 0, 30)
        fuse_us('front_left', 45, 30)
        fuse_us('front_right', -45, 30)
        fuse_us('left', 90, 30)
        fuse_us('right', -90, 30)

        # --- SAFETY BUBBLE ---
        # Find the absolute closest point
        min_idx = np.argmin(proc_ranges)
        min_dist = proc_ranges[min_idx]
        
        # If obstacle is close, zero out the array around it
        if min_dist < 1.0:
            radius_idx = int(math.radians(40) / angle_inc) # 40-degree bubble
            start = max(0, min_idx - radius_idx)
            end = min(len(proc_ranges), min_idx + radius_idx)
            proc_ranges[start:end] = 0.0 # Mark as "No Go Zone"

        return proc_ranges, min_dist

    def control_loop(self):
        if self.lidar_data is None: return
        
        twist = Twist()
        ranges = self.lidar_data.ranges
        amin = self.lidar_data.angle_min
        ainc = self.lidar_data.angle_increment
        
        # 1. PREPROCESS
        proc_ranges, closest_obj = self.preprocess_lidar(ranges, amin, ainc)

        # 2. EMERGENCY & RECOVERY LOGIC
        # If Front is critically blocked OR we are already reversing
        if self.reversing:
            self.reverse_timer += 1
            if self.reverse_timer > 30: # 1.5 seconds max reverse
                self.reversing = False
                self.reverse_timer = 0
            else:
                twist.linear.x = -0.15
                # Steer straight back or away from rear obstacles
                twist.angular.z = 0.0 
                self.cmd_pub.publish(twist)
                return

        if closest_obj < self.SAFETY_DIST:
            self.get_logger().warn("CRASH IMMINENT - REVERSING")
            self.reversing = True
            self.reverse_timer = 0
            return

        # 3. FIND MAX GAP (The F1Tenth Logic)
        # Find all rays that are "Deep" (further than threshold)
        # We consider anything > 1.0m as "Free Space"
        free_space = proc_ranges > self.GAP_THRESHOLD
        
        # Find consecutive Free Space slices
        # Zero-padding allows finding gaps at edges
        diff = np.diff(np.concatenate(([0], free_space.astype(int), [0])))
        starts = np.where(diff == 1)[0]
        ends = np.where(diff == -1)[0]
        
        best_gap = None
        max_gap_len = 0
        
        for s, e in zip(starts, ends):
            gap_len = e - s
            if gap_len > max_gap_len:
                max_gap_len = gap_len
                best_gap = (s, e)

        # 4. DRIVE
        if best_gap:
            s, e = best_gap
            
            # METHOD A: Aim for the Furthest Point in the Gap (Aggressive)
            # This pulls the robot out of corners fast
            gap_slice = proc_ranges[s:e]
            max_depth_idx = np.argmax(gap_slice)
            target_idx = s + max_depth_idx
            
            # Convert Index to Angle
            target_angle = amin + (target_idx * ainc)
            
            # STEERING
            # Gain = 1.0 (Standard)
            twist.angular.z = np.clip(target_angle * 1.2, -1.5, 1.5)
            
            # SPEED CONTROL (Don't stop on turns!)
            # If turning sharp (< 45 deg), keep speed high.
            # Only slow down for extreme turns (> 60 deg)
            turn_severity = abs(target_angle)
            if turn_severity < 0.5:
                twist.linear.x = self.MAX_SPEED
            else:
                # Minimum speed 0.1, never 0.0
                twist.linear.x = max(0.1, self.MAX_SPEED * (1.0 - turn_severity/2.0))
                
        else:
            # NO GAP FOUND (Trap) -> Spin
            self.get_logger().info("No Gap Found -> Spinning")
            twist.linear.x = 0.0
            twist.angular.z = 0.8

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = F1TenthNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()