import random

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32


class RandomSensor(Node):
    def __init__(self):
        super().__init__('random_sensor')

        # Declare parameters (so ros2 param get/list works AND launch overrides are accepted)
        self.declare_parameter('period', 0.5)
        self.declare_parameter('min_value', 0.0)
        self.declare_parameter('max_value', 100.0)

        self._timer = None
        self._read_params()

        self.publisher_ = self.create_publisher(Float32, '/random_value', 10)

        # Allow runtime parameter updates
        self.add_on_set_parameters_callback(self._on_params_changed)

        self._create_or_restart_timer()

        self.get_logger().info(
            f"RandomSensor started, period={self._period:.3f}s, "
            f"range=[{self._min_value}, {self._max_value}] -> topic /random_value"
        )

    def _read_params(self):
        self._period = float(self.get_parameter('period').value)
        self._min_value = float(self.get_parameter('min_value').value)
        self._max_value = float(self.get_parameter('max_value').value)

        # Safety
        if self._period <= 0.0:
            self._period = 0.5
        if self._max_value < self._min_value:
            self._min_value, self._max_value = self._max_value, self._min_value

    def _create_or_restart_timer(self):
        if self._timer is not None:
            self._timer.cancel()
        self._timer = self.create_timer(self._period, self._tick)

    def _tick(self):
        msg = Float32()
        msg.data = float(random.uniform(self._min_value, self._max_value))
        self.publisher_.publish(msg)

    def _on_params_changed(self, params):
        # Validate before applying
        new_period = self._period
        new_min = self._min_value
        new_max = self._max_value

        for p in params:
            if p.name == 'period':
                new_period = float(p.value)
            elif p.name == 'min_value':
                new_min = float(p.value)
            elif p.name == 'max_value':
                new_max = float(p.value)

        if new_period <= 0.0:
            return SetParametersResult(successful=False, reason='period must be > 0')

        # Normalize range
        if new_max < new_min:
            new_min, new_max = new_max, new_min

        # Apply
        self._period = new_period
        self._min_value = new_min
        self._max_value = new_max

        self._create_or_restart_timer()

        self.get_logger().info(
            f"Params updated: period={self._period:.3f}, range=[{self._min_value}, {self._max_value}]"
        )
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = RandomSensor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
