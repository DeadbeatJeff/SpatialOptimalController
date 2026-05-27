import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

class SafetyMitigationSubscriber(Node):
    def __init__(self):
        super().__init__('safety_mitigation')
        
        # Create a subscriber that listens to the 'ugv_telemetry' topic
        # It triggers 'listener_callback' every time a new message arrives
        self.subscription = self.create_subscription(
            Vector3,
            'ugv_telemetry',
            self.listener_callback,
            10)
        self.get_logger().info('Mitigation Brain Subscriber initialized. Monitoring telemetry...')

    def listener_callback(self, msg):
        roll = msg.x
        pitch = msg.y
        battery = msg.z
        
        # Engineering evaluation: Threshold checks for soldier safety
        if roll > 30.0:
            self.get_logger().error(f'CRITICAL WARNING: Roll angle ({roll:.1f}°) exceeds static stability limits! Threat of UGV Rollover!')
        elif battery < 15.0:
            self.get_logger().warn(f'LOW BATTERY ALERT: {battery:.1f}% remaining. Plan return to base.')
        else:
            self.get_logger().info(f'Telemetry verified nominal. UGV stable.')

def main(args=None):
    rclpy.init(args=args)
    node = SafetyMitigationSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()