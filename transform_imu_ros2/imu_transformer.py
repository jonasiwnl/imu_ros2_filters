from sensor_msgs.msg import Imu, MagneticField, Header
from tf2_ros import fromMsg, toMsg

import numpy as np


TRANSFORM = np.array([
    [0, 1, 0],
    [1, 0, 0],
    [0, 0, -1]
])

class ImuTransformer():
    def __init__(self, imu_publisher, mag_publisher, frame_id='imu'):
        self.imu_publisher = imu_publisher
        self.mag_publisher = mag_publisher
        self.frame_id = frame_id

    def _transform_header(self, header):
        transformed: Header = Header()

        transformed.seq = header.seq
        transformed.stamp = header.stamp
        transformed.frame_id = self.frame_id

        return transformed
    
    def _transform_spatial(self, data):
        converted = fromMsg(data)
        converted = TRANSFORM @ converted # Matmul
        return toMsg(converted)

    # covariance is always postive
    # 0 1 2    4 3 5
    # 3 4 5 -> 1 0 2
    # 6 7 8    7 6 8
    def _transform_matrix(self, matrix):
        return np.array([
            matrix[4], matrix[3], matrix[5],
            matrix[1], matrix[0], matrix[2],
            matrix[7], matrix[6], matrix[8],
        ])

    def transform_imu(self, imu_data):
        transformed: Imu = Imu()

        transformed.header = self.transform_header(imu_data.header)

        transformed.orientation = \
            self._transform_spatial(imu_data.orientation)
        transformed.orientation_covariance = \
            self._transform_matrix(imu_data.orientation_covariance)
        
        transformed.angular_velocity = \
            self._transform_spatial(imu_data.angular_velocity)
        transformed.angular_velocity_covariance = \
            self._transform_matrix(imu_data.angular_velocity_covariance)
        
        transformed.linear_acceleration = \
            self._transform_spatial(imu_data.linear_acceleration)
        transformed.linear_acceleration_covariance = \
            self._transform_matrix(imu_data.linear_acceleration_covariance)

        self.imu_publisher.publish(transformed)
    
    def transform_mag(self, mag_data):
        transformed: MagneticField = MagneticField()

        transformed.header = self.transform_header(mag_data.header)

        transformed.magnetic_field = \
            self._transform_spatial(mag_data.magnetic_field)
        transformed.magnetic_field_covariance = \
            self._transform_matrix(mag_data.magnetic_field_covariance)

        self.mag_publisher.publish(transformed)
