class ImuTransformer():
    def __init__(self, imu_publisher, mag_publisher):
        self.imu_publisher = imu_publisher
        self.mag_publisher = mag_publisher
        self.data = None

    def transform_imu(self, imu_data):
        # TODO
        return imu_data
    
    def transform_mag(self, mag_data):
        # TODO
        return mag_data
