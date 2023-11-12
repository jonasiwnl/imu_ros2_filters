import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class JitterFilter(Node):
    # last_stamp = (0, 0)
    diff = 0.1
    publisher = None
    subscriber = None
    prev_data = None

    def __init__(self):
        super().__init__("imu_jitter_filter_node")

        self.subscriber = self.create_subscription(
            Imu,
            "imu/data_filtered",
            self.filter_data,
            10
        )
        self.publisher = self.create_publisher(Imu, "imu/data_jitter_filtered", 10)

    def filter_data(self, data) -> None:
        self.get_logger().info("Filter data called", throttle_duration_sec=1.0)

        # for first function call. this sucks and needs to be cleaned
        if not self.prev_data:
            self.prev_data = data
            return

        lin_accel_ok = \
            self._filter(data.linear_acceleration, self.prev_data.linear_acceleration)
        ang_vel_ok = \
            self._filter(data.angular_velocity, self.prev_data.angular_velocity)
        
        if lin_accel_ok and ang_vel_ok:
            self.publisher.publish(data)

        self.prev_data = data

    def _filter(self, new_point, prev_point) -> bool:
        return abs(new_point.x - prev_point.x) >= self.diff and \
            abs(new_point.y - prev_point.y) >= self.diff and \
            abs(new_point.z - prev_point.z) >= self.diff

def main():
    rclpy.init()
    node = JitterFilter()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
