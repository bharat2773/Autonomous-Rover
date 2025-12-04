import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math

class CorridorCentering(Node):
    def __init__(self):
        super().__init__('corridor_centering')
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # --- TUNING ---
        self.max_speed = 0.30
        self.base_turn_speed = 1.5
        self.sensor_cap = 2.0
        self.stop_dist = 0.4
        self.stuck_threshold = 0.6  # If trapped by obstacles closer than this
        
        # --- STATE MACHINE ---
        self.state = "NAVIGATE"  # States: NAVIGATE, SCANNING, ROTATING
        self.scan_angles = []
        self.scan_distances = []
        self.best_angle = 0
        self.rotation_target = 0
        self.stuck_counter = 0
        self.rotation_complete = False

    def scan_callback(self, msg: LaserScan):
        twist = Twist()
        ranges = msg.ranges
        mid = len(ranges) // 2

        # Define slices
        front_slice = ranges[mid-10 : mid+10]
        left_slice  = ranges[mid+30 : mid+80]
        right_slice = ranges[mid-80 : mid-30]

        def get_dist(slice_data, cap_value):
            valid = [r for r in slice_data if not math.isinf(r) and not math.isnan(r) and r > 0.05]
            if not valid:
                return cap_value
            d = min(valid)
            return min(d, cap_value)

        d_front = get_dist(front_slice, self.sensor_cap)
        d_left  = get_dist(left_slice,  self.sensor_cap)
        d_right = get_dist(right_slice, self.sensor_cap)

        # === STATE MACHINE ===
        
        if self.state == "NAVIGATE":
            # Check if stuck (obstacles on multiple sides, close proximity)
            if d_front < self.stuck_threshold and d_left < self.stuck_threshold and d_right < self.stuck_threshold:
                self.stuck_counter += 1
                if self.stuck_counter > 3:  # Confirm stuck for 3 consecutive readings
                    self.get_logger().warn("🚨 STUCK DETECTED! Initiating 360° scan...")
                    self.state = "SCANNING"
                    self.scan_angles = []
                    self.scan_distances = []
                    self.stuck_counter = 0
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_pub.publish(twist)
                    return
            else:
                self.stuck_counter = 0

            # Emergency reverse
            if d_front < self.stop_dist:
                twist.linear.x = -0.10
                twist.angular.z = 0.0
                self.get_logger().warn("Too Close! Reversing...")
                self.cmd_pub.publish(twist)
                return

            # Normal navigation
            speed_factor = min(d_front / 1.5, 1.0)
            twist.linear.x = self.max_speed * speed_factor
            error = d_left - d_right
            twist.angular.z = error * self.base_turn_speed
            
            self.get_logger().info(
                f"F:{d_front:.2f} L:{d_left:.2f} R:{d_right:.2f} | Err:{error:.2f} | Spd:{twist.linear.x:.2f}"
            )

        elif self.state == "SCANNING":
            # Rotate slowly and record distances in all directions
            twist.linear.x = 0.0
            twist.angular.z = 0.8  # Slow rotation
            
            # Sample the environment (360-degree scan)
            self.scan_angles.append(len(self.scan_angles) * 5)  # Approximate angle
            self.scan_distances.append(d_front)
            
            # Complete full rotation (72 samples at 5° each = 360°)
            if len(self.scan_angles) >= 72:
                self.state = "ROTATING"
                self.find_best_path()
                self.rotation_complete = False
                self.get_logger().info(f"✅ Scan complete! Best path at {self.best_angle}°")

        elif self.state == "ROTATING":
            # Rotate to the best direction found
            if not self.rotation_complete:
                twist.linear.x = 0.0
                twist.angular.z = 0.6 if self.rotation_target > 0 else -0.6
                
                self.rotation_target -= 5 if self.rotation_target > 0 else -5
                
                if abs(self.rotation_target) < 10:
                    self.rotation_complete = True
                    self.get_logger().info("🎯 Rotation complete! Resuming navigation...")
                    self.state = "NAVIGATE"
            else:
                self.state = "NAVIGATE"

        self.cmd_pub.publish(twist)

    def find_best_path(self):
        """Analyze 360° scan and find the direction with most clearance"""
        if not self.scan_distances:
            self.best_angle = 0
            self.rotation_target = 0
            return
        
        # Find sector with maximum average distance (best clearance)
        sector_size = 10  # Average every 10 samples (50° sectors)
        best_score = 0
        best_idx = 0
        
        for i in range(0, len(self.scan_distances) - sector_size, 5):
            sector = self.scan_distances[i:i+sector_size]
            # Score: average distance + bonus for max distance
            avg_dist = sum(sector) / len(sector)
            max_dist = max(sector)
            score = avg_dist + 0.3 * max_dist
            
            if score > best_score:
                best_score = score
                best_idx = i
        
        # Calculate rotation needed
        self.best_angle = best_idx * 5  # Convert index to degrees
        self.rotation_target = self.best_angle - 180  # Adjust for current facing
        
        self.get_logger().info(f"Best clearance: {best_score:.2f}m at {self.best_angle}°")

def main(args=None):
    rclpy.init(args=args)
    node = CorridorCentering()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()