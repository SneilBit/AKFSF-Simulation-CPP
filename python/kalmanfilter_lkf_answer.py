# ------------------------------------------------------------------------------- #
# Advanced Kalman Filtering and Sensor Fusion Course - Linear Kalman Filter
#
# ####### ANSWER FILE #######
#
# Usage:
# -Rename this file to "kalmanfilter.py" if you want to use this code.

import math

import numpy as np

from kalmanfilter_base import *
from utils import *

# -------------------------------------------------- #
# YOU CAN USE AND MODIFY THESE CONSTANTS HERE
INIT_ON_FIRST_PREDICTION = False
INIT_POS_STD = 0
INIT_VEL_STD = 15
ACCEL_STD = 0.1
GPS_POS_STD = 3.0
# -------------------------------------------------- #


class KalmanFilter(KalmanFilterBase):

    def predictionStepDt(self, dt):
        if not self.isInitialised() and INIT_ON_FIRST_PREDICTION:
            # Implement the State Vector and Covariance Matrix Initialisation in the
            # section below if you want to initialise the filter WITHOUT waiting for
            # the first measurement to occur. Make sure you call the setState() /
            # setCovariance() functions once you have generated the initial conditions.
            # Hint: Assume the state vector has the form [X,Y,VX,VY].
            # Hint: You can use the constants: INIT_POS_STD, INIT_VEL_STD
            # ----------------------------------------------------------------------- #
            # ENTER YOUR CODE HERE
            state = np.zeros(4)
            cov = np.zeros((4, 4))

            # Assume the initial position is (X,Y) = (0,0) m
            # Assume the initial velocity is 5 m/s at 45 degrees (VX,VY) = (5*cos(45deg),5*sin(45deg)) m/s
            state[:] = [0, 0, 5.0 * math.cos(math.pi / 4), 5.0 * math.sin(math.pi / 4)]

            init_pos_std = INIT_POS_STD
            init_vel_std = INIT_VEL_STD
            cov[0, 0] = INIT_POS_STD * INIT_POS_STD
            cov[1, 1] = INIT_POS_STD * INIT_POS_STD
            cov[2, 2] = INIT_VEL_STD * INIT_VEL_STD
            cov[3, 3] = INIT_VEL_STD * INIT_VEL_STD

            self.setState(state)
            self.setCovariance(cov)
            # ----------------------------------------------------------------------- #

        if self.isInitialised():
            state = self.getState()
            cov = self.getCovariance()

            # Implement The Kalman Filter Prediction Step for the system in the
            # section below.
            # Hint: You can use the constants: ACCEL_STD
            # ----------------------------------------------------------------------- #
            # ENTER YOUR CODE HERE

            F = np.array([[1, 0, dt, 0],
                          [0, 1, 0, dt],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]], dtype=float)

            Q = np.zeros((2, 2))
            Q[0, 0] = (ACCEL_STD * ACCEL_STD)
            Q[1, 1] = (ACCEL_STD * ACCEL_STD)

            L = np.array([[(0.5 * dt * dt), 0],
                          [0, (0.5 * dt * dt)],
                          [dt, 0],
                          [0, dt]], dtype=float)

            state = F @ state
            cov = F @ cov @ F.T + L @ Q @ L.T

            # ----------------------------------------------------------------------- #

            self.setState(state)
            self.setCovariance(cov)

    def handleGPSMeasurement(self, meas):
        if self.isInitialised():
            state = self.getState()
            cov = self.getCovariance()

            # Implement The Kalman Filter Update Step for the GPS Measurements in the
            # section below.
            # Hint: Assume that the GPS sensor has a 3m (1 sigma) position uncertainty.
            # Hint: You can use the constants: GPS_POS_STD
            # ----------------------------------------------------------------------- #
            # ENTER YOUR CODE HERE

            z = np.array([meas.x, meas.y], dtype=float)
            H = np.array([[1, 0, 0, 0],
                          [0, 1, 0, 0]], dtype=float)
            R = np.zeros((2, 2))
            R[0, 0] = GPS_POS_STD * GPS_POS_STD
            R[1, 1] = GPS_POS_STD * GPS_POS_STD

            z_hat = H @ state
            y = z - z_hat
            S = H @ cov @ H.T + R
            K = cov @ H.T @ inverse(S)

            state = state + K @ y
            cov = (np.identity(4) - K @ H) @ cov

            # ----------------------------------------------------------------------- #

            self.setState(state)
            self.setCovariance(cov)
        else:
            # Implement the State Vector and Covariance Matrix Initialisation in the
            # section below. Make sure you call the setState/setCovariance functions
            # once you have generated the initial conditions.
            # Hint: Assume the state vector has the form [X,Y,VX,VY].
            # Hint: You can use the constants: GPS_POS_STD, INIT_VEL_STD
            # ----------------------------------------------------------------------- #
            # ENTER YOUR CODE HERE
            state = np.zeros(4)
            cov = np.zeros((4, 4))

            state[0] = meas.x
            state[1] = meas.y
            cov[0, 0] = GPS_POS_STD * GPS_POS_STD
            cov[1, 1] = GPS_POS_STD * GPS_POS_STD
            cov[2, 2] = INIT_VEL_STD * INIT_VEL_STD
            cov[3, 3] = INIT_VEL_STD * INIT_VEL_STD

            self.setState(state)
            self.setCovariance(cov)
            # ----------------------------------------------------------------------- #

    def getVehicleStatePositionCovariance(self):
        pos_cov = np.zeros((2, 2))
        cov = self.getCovariance()
        if self.isInitialised() and cov.size != 0:
            pos_cov = np.array([[cov[0, 0], cov[0, 1]], [cov[1, 0], cov[1, 1]]])
        return pos_cov

    def getVehicleState(self):
        if self.isInitialised():
            state = self.getState()  # STATE VECTOR [X,Y,VX,VY]
            psi = math.atan2(state[3], state[2])
            V = math.sqrt(state[2] * state[2] + state[3] * state[3])
            return VehicleState(state[0], state[1], psi, V)
        return VehicleState()

    def predictionStepGyro(self, gyro, dt):
        self.predictionStep(dt)

    def handleLidarMeasurements(self, dataset, map):
        pass

    def handleLidarMeasurement(self, meas, map):
        pass
