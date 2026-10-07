# ------------------------------------------------------------------------------- #
# Advanced Kalman Filtering and Sensor Fusion Course - Extended Kalman Filter
#
# ####### STUDENT FILE #######
#
# Usage:
# -Rename this file to "kalmanfilter.py" if you want to use this code.

import math

import numpy as np

from kalmanfilter_base import *
from utils import *

# -------------------------------------------------- #
# YOU CAN USE AND MODIFY THESE CONSTANTS HERE
ACCEL_STD = 1.0
GYRO_STD = 0.01 / 180.0 * math.pi
INIT_VEL_STD = 10.0
INIT_PSI_STD = 45.0 / 180.0 * math.pi
GPS_POS_STD = 3.0
LIDAR_RANGE_STD = 3.0
LIDAR_THETA_STD = 0.02
# -------------------------------------------------- #


class KalmanFilter(KalmanFilterBase):

    def handleLidarMeasurements(self, dataset, map):
        # Assume No Correlation between the Measurements and Update Sequentially
        for meas in dataset:
            self.handleLidarMeasurement(meas, map)

    def handleLidarMeasurement(self, meas, map):
        if self.isInitialised():
            state = self.getState()
            cov = self.getCovariance()

            # Implement The Kalman Filter Update Step for the Lidar Measurements in the
            # section below.
            # HINT: use the wrapAngle() function on angular values to always keep angle
            # values within correct range, otherwise strange angle effects might be seen.
            # HINT: You can use the constants: LIDAR_RANGE_STD, LIDAR_THETA_STD
            # HINT: The mapped-matched beacon position can be accessed by the variables
            # map_beacon.x and map_beacon.y
            # ----------------------------------------------------------------------- #
            # ENTER YOUR CODE HERE

            map_beacon = map.getBeaconWithId(meas.id)  # Match Beacon with built in Data Association Id
            if meas.id != -1 and map_beacon.id != -1:
                # The map matched beacon positions can be accessed using: map_beacon.x AND map_beacon.y
                pass

            # ----------------------------------------------------------------------- #

            self.setState(state)
            self.setCovariance(cov)

    def predictionStepGyro(self, gyro, dt):
        if self.isInitialised():
            state = self.getState()
            cov = self.getCovariance()

            # Implement The Kalman Filter Prediction Step for the system in the
            # section below.
            # HINT: Assume the state vector has the form [PX, PY, PSI, V].
            # HINT: Use the Gyroscope measurement as an input into the prediction step.
            # HINT: You can use the constants: ACCEL_STD, GYRO_STD
            # HINT: use the wrapAngle() function on angular values to always keep angle
            # values within correct range, otherwise strange angle effects might be seen.
            # ----------------------------------------------------------------------- #
            # ENTER YOUR CODE HERE

            # ----------------------------------------------------------------------- #

            self.setState(state)
            self.setCovariance(cov)

    def handleGPSMeasurement(self, meas):
        # All this code is the same as the LKF as the measurement model is linear
        # so the EKF update state would just produce the same result.
        if self.isInitialised():
            state = self.getState()
            cov = self.getCovariance()

            z = np.zeros(2)
            H = np.zeros((2, 4))
            R = np.zeros((2, 2))

            z[:] = [meas.x, meas.y]
            H[:] = [[1, 0, 0, 0],
                    [0, 1, 0, 0]]
            R[0, 0] = GPS_POS_STD * GPS_POS_STD
            R[1, 1] = GPS_POS_STD * GPS_POS_STD

            z_hat = H @ state
            y = z - z_hat
            S = H @ cov @ H.T + R
            K = cov @ H.T @ inverse(S)

            state = state + K @ y
            cov = (np.identity(4) - K @ H) @ cov

            self.setState(state)
            self.setCovariance(cov)
        else:
            state = np.zeros(4)
            cov = np.zeros((4, 4))

            state[0] = meas.x
            state[1] = meas.y
            cov[0, 0] = GPS_POS_STD * GPS_POS_STD
            cov[1, 1] = GPS_POS_STD * GPS_POS_STD
            cov[2, 2] = INIT_PSI_STD * INIT_PSI_STD
            cov[3, 3] = INIT_VEL_STD * INIT_VEL_STD

            self.setState(state)
            self.setCovariance(cov)

    def getVehicleStatePositionCovariance(self):
        pos_cov = np.zeros((2, 2))
        cov = self.getCovariance()
        if self.isInitialised() and cov.size != 0:
            pos_cov = np.array([[cov[0, 0], cov[0, 1]], [cov[1, 0], cov[1, 1]]])
        return pos_cov

    def getVehicleState(self):
        if self.isInitialised():
            state = self.getState()  # STATE VECTOR [X,Y,PSI,V,...]
            return VehicleState(state[0], state[1], state[2], state[3])
        return VehicleState()

    def predictionStepDt(self, dt):
        pass
