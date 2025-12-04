import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        # ---------- FIX: Use BEST_EFFORT QoS ----------
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        # ------------------------------------------------

        # Subscribe to /scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos
        )

        # Publisher to /cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.stop_distance = 0.5  # meters

    def scan_callback(self, msg):
        ranges = msg.ranges
        mid = len(ranges) // 2
        front_ranges = ranges[mid-30 : mid+30]

        # Remove invalid readings
        front_ranges = [r for r in front_ranges if r > 0.01]

        min_dist = min(front_ranges) if front_ranges else 10.0

        self.get_logger().info(f"Front min distance: {min_dist:.2f} m")

        twist = Twist()

        if min_dist < self.stop_distance:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().warn("Obstacle detected! Stopping!")
        else:
            twist.linear.x = 0.2
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
