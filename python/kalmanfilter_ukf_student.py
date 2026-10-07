# ------------------------------------------------------------------------------- #
# Advanced Kalman Filtering and Sensor Fusion Course - Unscented Kalman Filter
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
ACCEL_STD = 0.05
GYRO_STD = 0.01 / 180.0 * math.pi
INIT_VEL_STD = 2
INIT_PSI_STD = 5.0 / 180.0 * math.pi
GPS_POS_STD = 3.0
LIDAR_RANGE_STD = 3.0
LIDAR_THETA_STD = 0.02
# -------------------------------------------------- #


# ----------------------------------------------------------------------- #
# USEFUL HELPER FUNCTIONS
def normaliseState(state):
    state = np.array(state, dtype=float, copy=True)
    state[2] = wrapAngle(state[2])
    return state


def normaliseLidarMeasurement(meas):
    meas = np.array(meas, dtype=float, copy=True)
    meas[1] = wrapAngle(meas[1])
    return meas


def generateSigmaPoints(state, cov):
    sigmaPoints = []

    # ----------------------------------------------------------------------- #
    # ENTER YOUR CODE HERE

    # ----------------------------------------------------------------------- #

    return sigmaPoints


def generateSigmaWeights(numStates):
    weights = []

    # ----------------------------------------------------------------------- #
    # ENTER YOUR CODE HERE

    # ----------------------------------------------------------------------- #

    return weights


def lidarMeasurementModel(aug_state, beaconX, beaconY):
    z_hat = np.zeros(2)

    # ----------------------------------------------------------------------- #
    # ENTER YOUR CODE HERE

    # ----------------------------------------------------------------------- #

    return z_hat


def vehicleProcessModel(aug_state, psi_dot, dt):
    new_state = np.zeros(4)

    # ----------------------------------------------------------------------- #
    # ENTER YOUR CODE HERE

    # ----------------------------------------------------------------------- #

    return new_state
# ----------------------------------------------------------------------- #


class KalmanFilter(KalmanFilterBase):

    def handleLidarMeasurement(self, meas, map):
        if self.isInitialised():
            state = self.getState()
            cov = self.getCovariance()

            # Implement The Kalman Filter Update Step for the Lidar Measurements in the
            # section below.
            # HINT: Use the normaliseState() and normaliseLidarMeasurement() functions
            # to always keep angle values within correct range.
            # HINT: Do not normalise during sigma point calculation!
            # HINT: You can use the constants: LIDAR_RANGE_STD, LIDAR_THETA_STD
            # HINT: The mapped-matched beacon position can be accessed by the variables
            # map_beacon.x and map_beacon.y
            # ----------------------------------------------------------------------- #
            # ENTER YOUR CODE HERE

            map_beacon = map.getBeaconWithId(meas.id)  # Match Beacon with built in Data Association Id
            if meas.id != -1 and map_beacon.id != -1:  # Check that we have a valid beacon match
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
            # HINT: Use the normaliseState() function to always keep angle values within correct range.
            # HINT: Do NOT normalise during sigma point calculation!
            # ----------------------------------------------------------------------- #
            # ENTER YOUR CODE HERE


            # ----------------------------------------------------------------------- #

            self.setState(state)
            self.setCovariance(cov)

    def handleGPSMeasurement(self, meas):
        # All this code is the same as the LKF as the measurement model is linear
        # so the UKF update state would just produce the same result.
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
            # You may modify this initialisation routine if you can think of a more
            # robust and accuracy way of initialising the filter.
            # ----------------------------------------------------------------------- #
            # YOU ARE FREE TO MODIFY THE FOLLOWING CODE HERE

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

            # ----------------------------------------------------------------------- #

    def handleLidarMeasurements(self, dataset, map):
        # Assume No Correlation between the Measurements and Update Sequentially
        for meas in dataset:
            self.handleLidarMeasurement(meas, map)

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
