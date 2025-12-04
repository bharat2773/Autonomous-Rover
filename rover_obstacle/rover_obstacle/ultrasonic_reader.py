import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan  # CHANGED from Range
from rclpy.qos import QoSProfile, ReliabilityPolicy

class UltrasonicReader(Node):
    def __init__(self):
        super().__init__('ultrasonic_reader')

        # Use Best Effort to match Gazebo
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.topics = [
            "/us_sensor/front", "/us_sensor/front_left", "/us_sensor/left",
            "/us_sensor/rear_left", "/us_sensor/rear", "/us_sensor/rear_right",
            "/us_sensor/right", "/us_sensor/front_right"
        ]

        self.values = {topic: None for topic in self.topics}

        for topic in self.topics:
            self.create_subscription(
                LaserScan,  # Subscribe to LaserScan (Array)
                topic,
                self.make_callback(topic),
                qos
            )

        self.create_timer(0.2, self.display_values)

    def make_callback(self, topic):
        def callback(msg):
            # msg.ranges is the ARRAY of 10 rays
            # We want the CLOSEST point in that array to detect obstacles
            
            # Filter out infinite values (no object detected)
            valid_readings = [r for r in msg.ranges if r > msg.range_min and r < msg.range_max]
            
            if valid_readings:
                # Store the minimum distance found in the cone
                self.values[topic] = min(valid_readings)
            else:
                self.values[topic] = float('inf') # Nothing in the cone
                
        return callback

    def display_values(self):
        output = " | ".join(
            f"{name.split('/')[-1]}: {dist:.2f}m" if dist != float('inf') and dist is not None else f"{name.split('/')[-1]}: ---"
            for name, dist in self.values.items()
        )
        self.get_logger().info(output)

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()