import math

from cpp_random import mt19937, normal_distribution, uniform_real_distribution
from utils import wrapAngle


class GPSMeasurement:
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y


class GyroMeasurement:
    def __init__(self, psi_dot=0.0):
        self.psi_dot = psi_dot


class LidarMeasurement:
    def __init__(self, range=0.0, theta=0.0, id=0):
        self.range = range
        self.theta = theta
        self.id = id


# GPS Sensor
class GPSSensor:
    def __init__(self):
        self.m_rand_gen = mt19937()
        self.m_noise_std = 0.0
        self.m_error_prob = 0.0
        self.m_gps_denied_x = 0.0
        self.m_gps_denied_y = 0.0
        self.m_gps_denied_range = -1.0

    def reset(self):
        self.m_rand_gen = mt19937()

    def setGPSNoiseStd(self, std):
        self.m_noise_std = std

    def setGPSErrorProb(self, prob):
        self.m_error_prob = prob

    def setGPSDeniedZone(self, x, y, r):
        self.m_gps_denied_x = x
        self.m_gps_denied_y = y
        self.m_gps_denied_range = r

    def generateGPSMeasurement(self, sensor_x, sensor_y):
        meas = GPSMeasurement()
        gps_pos_dis = normal_distribution(0.0, self.m_noise_std)
        gps_error_dis = uniform_real_distribution(0.0, 1.0)
        meas.x = sensor_x + gps_pos_dis(self.m_rand_gen)
        meas.y = sensor_y + gps_pos_dis(self.m_rand_gen)
        if gps_error_dis(self.m_rand_gen) < self.m_error_prob:
            meas.x = 0
            meas.y = 0
        delta_x = sensor_x - self.m_gps_denied_x
        delta_y = sensor_y - self.m_gps_denied_y
        range = math.sqrt(delta_x * delta_x + delta_y * delta_y)
        if range < self.m_gps_denied_range:
            meas.x = 0
            meas.y = 0
        return meas


# Gyro Sensor
class GyroSensor:
    def __init__(self):
        self.m_rand_gen = mt19937()
        self.m_noise_std = 0.0
        self.m_bias = 0.0

    def reset(self):
        self.m_rand_gen = mt19937()

    def setGyroNoiseStd(self, std):
        self.m_noise_std = std

    def setGyroBias(self, bias):
        self.m_bias = bias

    def generateGyroMeasurement(self, sensor_yaw_rate):
        meas = GyroMeasurement()
        gyro_dis = normal_distribution(0.0, self.m_noise_std)
        meas.psi_dot = sensor_yaw_rate + self.m_bias + gyro_dis(self.m_rand_gen)
        return meas


# Lidar Sensor
class LidarSensor:
    def __init__(self):
        self.m_rand_gen = mt19937()
        self.m_range_noise_std = 0.0
        self.m_theta_noise_std = 0.0
        self.m_max_range = 90.0
        self.m_id_enabled = True

    def reset(self):
        self.m_rand_gen = mt19937()

    def setLidarNoiseStd(self, range_std, theta_std):
        self.m_range_noise_std = range_std
        self.m_theta_noise_std = theta_std

    def setLidarMaxRange(self, range):
        self.m_max_range = range

    def setLidarDAEnabled(self, id_enabled):
        self.m_id_enabled = id_enabled

    def generateLidarMeasurements(self, sensor_x, sensor_y, sensor_yaw, map):
        meas = []
        lidar_theta_dis = normal_distribution(0.0, self.m_theta_noise_std)
        lidar_range_dis = normal_distribution(0.0, self.m_range_noise_std)
        for beacon in map.getBeacons():
            delta_x = beacon.x - sensor_x
            delta_y = beacon.y - sensor_y
            theta = wrapAngle(math.atan2(delta_y, delta_x) - sensor_yaw)
            beacon_range = math.sqrt(delta_x * delta_x + delta_y * delta_y)
            if beacon_range < self.m_max_range:
                beacon_meas = LidarMeasurement()
                beacon_meas.range = abs(beacon_range + lidar_range_dis(self.m_rand_gen))
                beacon_meas.theta = wrapAngle(theta + lidar_theta_dis(self.m_rand_gen))
                beacon_meas.id = (beacon.id if self.m_id_enabled else -1)
                meas.append(beacon_meas)
        return meas
