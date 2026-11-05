"""ROS 2 node that adds filtered camera-pixel coordinates to an existing Kaiju graph.

Original team camera publisher: /cv_coords (std_msgs/Float32, x 0..640)
New output: /cv_coords_filtered (std_msgs/Float32, still pixels 0..640)
Additional outputs: /cv_velocity_px_s (Float32), /cv_target_visible (Bool)
The Zumo subscriber must be switched to /cv_coords_filtered explicitly later.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

from kaiju_target_tracking.pixel_kalman import PixelKalman


class OpponentKalmanNode(Node):
    def __init__(self):
        super().__init__('opponent_kalman')
        self.tracker = PixelKalman(width_px=640.0)
        self.sub = self.create_subscription(
            Float32, '/cv_coords', self.on_camera_coordinate, 10)
        self.pixel_pub = self.create_publisher(Float32, '/cv_coords_filtered', 10)
        self.velocity_pub = self.create_publisher(Float32, '/cv_velocity_px_s', 10)
        self.visible_pub = self.create_publisher(Bool, '/cv_target_visible', 10)
        self.timer = self.create_timer(0.1, self.publish_estimate)
        self.get_logger().info('Filtering /cv_coords -> /cv_coords_filtered')

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_camera_coordinate(self, msg):
        # Original publisher skips frames where no opponent was detected.
        self.tracker.observe(float(msg.data), self._now_s())

    def publish_estimate(self):
        estimate = self.tracker.estimate(self._now_s())
        self.visible_pub.publish(Bool(data=(estimate is not None)))
        if estimate is None:
            return  # Don't pass old target coordinates as if they were current.
        self.pixel_pub.publish(Float32(data=float(estimate.x_px)))
        self.velocity_pub.publish(Float32(data=float(estimate.vx_px_s)))


def main(args=None):
    rclpy.init(args=args)
    node = OpponentKalmanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
