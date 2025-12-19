from collections import deque

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32


class RandomProcessor(Node):
    def __init__(self):
        super().__init__('random_processor')

        # Declare parameters (so launch overrides are accepted)
        self.declare_parameter('window_size', 10)
        self.declare_parameter('scale_factor', 1.0)

        self._read_params()
        self.buffer = deque(maxlen=self._window_size)

        self.sub_ = self.create_subscription(Float32, '/random_value', self._cb, 10)
        self.pub_ = self.create_publisher(Float32, '/processed_value', 10)

        self.add_on_set_parameters_callback(self._on_params_changed)

        self.get_logger().info(
            f"RandomProcessor ready: /random_value -> /processed_value, "
            f"window_size={self._window_size}, scale_factor={self._scale_factor}"
        )

    def _read_params(self):
        ws = int(self.get_parameter('window_size').value)
        sf = float(self.get_parameter('scale_factor').value)
        if ws < 1:
            ws = 1
        self._window_size = ws
        self._scale_factor = sf

    def _on_params_changed(self, params):
        new_ws = self._window_size
        new_sf = self._scale_factor

        for p in params:
            if p.name == 'window_size':
                new_ws = int(p.value)
            elif p.name == 'scale_factor':
                new_sf = float(p.value)

        if new_ws < 1:
            return SetParametersResult(successful=False, reason='window_size must be >= 1')

        # Apply
        self._window_size = new_ws
        self._scale_factor = new_sf

        # Recreate buffer with new maxlen, keep latest values
        old = list(self.buffer)
        self.buffer = deque(old, maxlen=self._window_size)

        self.get_logger().info(
            f"Params updated: window_size={self._window_size}, scale_factor={self._scale_factor}"
        )
        return SetParametersResult(successful=True)

    def _cb(self, msg: Float32):
        self.buffer.append(float(msg.data))
        avg = sum(self.buffer) / len(self.buffer)
        out = Float32()
        out.data = float(avg * self._scale_factor)
        self.pub_.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = RandomProcessor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
