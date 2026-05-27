import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import random

class SafetyMonitorPublisher(Node):
    def __init__(self):
        super().__init__('safety_monitor')
        
        # Create a publisher that sends Vector3 messages over a topic named 'ugv_telemetry'
        # The '10' is the QoS (Quality of Service) queue size
        self.publisher_ = self.create_publisher(Vector3, 'ugv_telemetry', 10)
        
        # Publish every 0.5 seconds for higher fidelity tracking
        self.timer = self.create_timer(0.5, self.publish_telemetry)
        self.get_logger().info('Telemetry Publisher Node initialized.')
        
        # Simulated initial state
        self.battery = 100.0

    def publish_telemetry(self):
        msg = Vector3()
        
        # Simulate mechanical dynamics (small random fluctuations in vehicle attitude)
        msg.x = random.uniform(-15.0, 35.0)  # Roll angle in degrees (simulating a slope)
        msg.y = random.uniform(-5.0, 12.0)   # Pitch angle in degrees
        
        # Slowly deplete battery
        self.battery -= 0.1
        msg.z = max(self.battery, 0.0)
        
        # Broadcast the data to the ROS2 network
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing Telemetry -> Roll: {msg.x:.1f}°, Pitch: {msg.y:.1f}°, Batt: {msg.z:.1f}%')

def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()