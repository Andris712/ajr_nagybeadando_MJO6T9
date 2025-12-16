#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class RandomProcessor(Node):
    def __init__(self):
        super().__init__('random_processor')
        self.sub = self.create_subscription(Float32, 'random_value', self.cb, 10)
        self.pub = self.create_publisher(Float32, 'processed_value', 10)
        self.get_logger().info('RandomProcessor ready: /random_value -> /processed_value')

    def cb(self, msg: Float32):
        out = Float32()
        # tetszőleges feldolgozás – itt csak 1.5-szörözzük és kerekítünk
        out.data = round(msg.data * 1.5, 4)
        self.pub.publish(out)

def main():
    rclpy.init()
    node = RandomProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
