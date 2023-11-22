import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class TemporalFilter(Node):
    def __init__(self):
        super().__init__('imu_temporal_filter_node')

        self.last_stamp = (0, 0)
        self.subscriber = self.create_subscription(
            Imu,
            # This topic is wrong
            'bno055/imu',
            self.filter_data,
            10
        )
        self.publisher = self.create_publisher(Imu, 'imu/data_temporal_filtered', 10)

    def filter_data(self, data) -> None:
        now = self.get_clock().now()
        stamp = data.header.stamp

        if stamp < self.last_stamp or stamp > now:
            return
        
        self.publisher.publish(data)
        self.last_stamp = stamp

def main():
    rclpy.init()
    node = TemporalFilter()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
