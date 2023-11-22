import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

import imu_transformer


def main():
    rclpy.init()
    node = Node('transform_imu_node')

    imu_publisher = node.create_publisher(Imu, 'imu/data_raw_enu', 10)
    mag_publisher = node.create_publisher(MagneticField, 'imu/mag_endu', 10)

    transformer = imu_transformer.ImuTransformer(imu_publisher, mag_publisher)

    # imu_subscriber =
    node.create_subscription(
        Imu,
        'imu/data_raw',
        transformer.transform_imu,
        10
    )
    # mag_subscriber =
    node.create_subscription(
        MagneticField,
        'imu/mag',
        transformer.transform_mag,
        10
    )

    rclpy.spin(node)

if __name__ == '__main__':
    main()
