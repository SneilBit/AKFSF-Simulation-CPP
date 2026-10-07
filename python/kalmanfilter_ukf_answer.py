# ------------------------------------------------------------------------------- #
# Advanced Kalman Filtering and Sensor Fusion Course - Unscented Kalman Filter
#
# ####### ANSWER FILE #######
#
# Usage:
# -Rename this file to "kalmanfilter.py" if you want to use this code.

import math

import numpy as np

from kalmanfilter_base import *
from utils import *

# ----------------------------------------------------------------------- #
# YOU CAN USE AND MODIFY THESE CONSTANTS HERE
ACCEL_STD = 1.0
GYRO_STD = 0.01 / 180.0 * math.pi
INIT_VEL_STD = 10.0
INIT_PSI_STD = 45.0 / 180.0 * math.pi
GPS_POS_STD = 3.0
LIDAR_RANGE_STD = 3.0
LIDAR_THETA_STD = 0.02
# ----------------------------------------------------------------------- #


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

    numStates = state.size
    lambda_ = 3.0 - numStates
    sqrtCov = llt_matrixL(cov)
    sigmaPoints.append(state)
    for iState in range(numStates):
        sigmaPoints.append(state + math.sqrt(lambda_ + numStates) * sqrtCov[:, iState])
        sigmaPoints.append(state - math.sqrt(lambda_ + numStates) * sqrtCov[:, iState])

    # ----------------------------------------------------------------------- #

    return sigmaPoints


def generateSigmaWeights(numStates):
    weights = []

    # ----------------------------------------------------------------------- #
    # ENTER YOUR CODE HERE

    lambda_ = 3.0 - numStates
    w0 = lambda_ / (lambda_ + numStates)
    wi = 0.5 / (numStates + lambda_)
    weights.append(w0)
    for i in range(2 * numStates):
        weights.append(wi)

    # ----------------------------------------------------------------------- #

    return weights


def lidarMeasurementModel(aug_state, beaconX, beaconY):
    z_hat = np.zeros(2)

    # ----------------------------------------------------------------------- #
    # ENTER YOUR CODE HERE

    x = aug_state[0]
    y = aug_state[1]
    psi = aug_state[2]
    range_noise = aug_state[4]
    theta_noise = aug_state[5]

    delta_x = beaconX - x
    delta_y = beaconY - y
    zhat_range = math.sqrt(delta_x * delta_x + delta_y * delta_y) + range_noise
    zhat_theta = math.atan2(delta_y, delta_x) - aug_state[2] + theta_noise
    z_hat[:] = [zhat_range, zhat_theta]

    # ----------------------------------------------------------------------- #

    return z_hat


def vehicleProcessModel(aug_state, psi_dot, dt):
    new_state = np.zeros(4)

    # ----------------------------------------------------------------------- #
    # ENTER YOUR CODE HERE

    x = aug_state[0]
    y = aug_state[1]
    psi = aug_state[2]
    V = aug_state[3]
    psi_dot_noise = aug_state[4]
    accel_noise = aug_state[5]

    x_new = x + dt * V * math.cos(psi)
    y_new = y + dt * V * math.sin(psi)
    psi_new = psi + dt * (psi_dot + psi_dot_noise)
    V_new = V + dt * accel_noise
    new_state[:] = [x_new, y_new, psi_new, V_new]

    # ----------------------------------------------------------------------- #

    return new_state


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
            # HINT: Use the normaliseState() and normaliseLidarMeasurement() functions
            # to always keep angle values within correct range.
            # HINT: Do not normalise during sigma point calculation!
            # HINT: You can use the constants: LIDAR_RANGE_STD, LIDAR_THETA_STD
            # HINT: The mapped-matched beacon position can be accessed by the variables
            # map_beacon.x and map_beacon.y
            # ----------------------------------------------------------------------- #
            # ENTER YOUR CODE HERE

            map_beacon = map.getBeaconWithId(meas.id)  # Match Beacon with built in Data Association Id
            if meas.id != -1 and map_beacon.id != -1:
                # Generate Measurement Vector
                z = np.zeros(2)
                z[:] = [meas.range, meas.theta]

                # Generate Measurement Model Noise Covariance Matrix
                R = np.zeros((2, 2))
                R[0, 0] = LIDAR_RANGE_STD * LIDAR_RANGE_STD
                R[1, 1] = LIDAR_THETA_STD * LIDAR_THETA_STD

                # Augment the State Vector with Noise States
                n_x = state.size
                n_v = 2
                n_z = 2
                n_aug = n_x + n_v
                x_aug = np.zeros(n_aug)
                P_aug = np.zeros((n_aug, n_aug))
                x_aug[:n_x] = state
                P_aug[:n_x, :n_x] = cov
                P_aug[n_aug - n_v:, n_aug - n_v:] = R

                # Generate Augmented Sigma Points
                sigma_points = generateSigmaPoints(x_aug, P_aug)
                sigma_weights = generateSigmaWeights(n_aug)

                # Measurement Model Augmented Sigma Points
                z_sig = []
                for sigma_point in sigma_points:
                    z_sig.append(lidarMeasurementModel(sigma_point, map_beacon.x, map_beacon.y))

                # Calculate Measurement Mean
                z_mean = np.zeros(n_z)
                for i in range(len(z_sig)):
                    z_mean += sigma_weights[i] * z_sig[i]

                # Calculate Innovation Covariance
                Py = np.zeros((n_z, n_z))
                for i in range(len(z_sig)):
                    diff = normaliseLidarMeasurement(z_sig[i] - z_mean)
                    Py += sigma_weights[i] * np.outer(diff, diff)

                # Calculate Cross Covariance
                # (loop bound kept exactly as in the C++ answer file: 2*n_x + 1, not 2*n_aug + 1)
                Pxy = np.zeros((n_x, n_z))
                for i in range(2 * n_x + 1):
                    x_diff = normaliseState(sigma_points[i][:n_x] - state)
                    z_diff = normaliseLidarMeasurement(z_sig[i] - z_mean)
                    Pxy += sigma_weights[i] * np.outer(x_diff, z_diff)

                K = Pxy @ inverse(Py)
                y = normaliseLidarMeasurement(z - z_mean)
                state = state + K @ y
                cov = cov - K @ Py @ K.T
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

            # Generate Q Matrix
            Q = np.zeros((2, 2))
            Q[0, 0] = GYRO_STD * GYRO_STD
            Q[1, 1] = ACCEL_STD * ACCEL_STD

            # Augment the State Vector with Noise States
            n_x = state.size
            n_w = 2
            n_aug = n_x + n_w
            x_aug = np.zeros(n_aug)
            P_aug = np.zeros((n_aug, n_aug))
            x_aug[:n_x] = state
            P_aug[:n_x, :n_x] = cov
            P_aug[n_aug - n_w:, n_aug - n_w:] = Q

            # Generate Augmented Sigma Points
            sigma_points = generateSigmaPoints(x_aug, P_aug)
            sigma_weights = generateSigmaWeights(n_aug)

            # Predict Augmented Sigma Points
            sigma_points_predict = []
            for sigma_point in sigma_points:
                sigma_points_predict.append(vehicleProcessModel(sigma_point, gyro.psi_dot, dt))

            # Calculate Mean
            state = np.zeros(n_x)
            for i in range(len(sigma_points_predict)):
                state += sigma_weights[i] * sigma_points_predict[i]
            state = normaliseState(state)

            # Calculate Covariance
            cov = np.zeros((n_x, n_x))
            for i in range(len(sigma_points_predict)):
                diff = normaliseState(sigma_points_predict[i] - state)
                cov += sigma_weights[i] * np.outer(diff, diff)

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
