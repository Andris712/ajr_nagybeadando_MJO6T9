import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class RandomSensor(Node):
    def __init__(self):
        super().__init__('random_sensor')
        self.declare_parameter('period', 0.5)  # másodperc
        period = float(self.get_parameter('period').value)
        self.pub = self.create_publisher(Float32, 'random_value', 10)
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().info(f'RandomSensor started, period={period}s -> topic /random_value')

    def _tick(self):
        msg = Float32()
        msg.data = random.uniform(0.0, 100.0)
        self.pub.publish(msg)
        self.get_logger().debug(f'sensor -> {msg.data:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = RandomSensor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
