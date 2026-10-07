# ------------------------------------------------------------------------------- #
# Advanced Kalman Filtering and Sensor Fusion Course - Linear Kalman Filter
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
INIT_ON_FIRST_PREDICTION = True
INIT_POS_STD = 0.0
INIT_VEL_STD = 0.0
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
            # Step 8: initial state restored to the true starting state
            # (Step 7 used an all-zero state)
            state[:] = [0, 0, 5.0 * math.cos(math.pi / 4), 5.0 * math.sin(math.pi / 4)]

            # Initial position uncertainty (Step 6 used INIT_POS_STD = 5 m)
            cov[0, 0] = INIT_POS_STD ** 2
            cov[1, 1] = INIT_POS_STD ** 2

            # Initial velocity uncertainty (Step 7 used INIT_VEL_STD = 5/3 m/s)
            cov[2, 2] = INIT_VEL_STD ** 2
            cov[3, 3] = INIT_VEL_STD ** 2

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
            # State vector: [X, Y, VX, VY]
            # Process model (constant velocity):
            #   x_k = F * x_{k-1}
            # State transition matrix F: position advances by velocity * dt,
            # velocity is assumed constant over the time step.
            #   X_k  = X_{k-1} + VX_{k-1} * dt
            #   Y_k  = Y_{k-1} + VY_{k-1} * dt
            #   VX_k = VX_{k-1}
            #   VY_k = VY_{k-1}
            F = np.array([[1, 0, dt, 0],
                          [0, 1, 0, dt],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])

            # Propagate the state estimate forward by dt
            state = F @ state

            # Process noise: acceleration is modelled as zero-mean noise with
            # std ACCEL_STD in each axis.
            # Q = [ax_variance, 0; 0, ay_variance]
            Q = np.array([[ACCEL_STD ** 2, 0],
                          [0, ACCEL_STD ** 2]])

            # L maps the acceleration noise into the state space:
            # position is affected by 0.5*a*dt^2, velocity by a*dt
            L = np.array([[0.5 * dt ** 2, 0],
                          [0, 0.5 * dt ** 2],
                          [dt, 0],
                          [0, dt]])

            # Propagate the covariance: P_k- = F * P_{k-1}+ * F^T + L * Q * L^T
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
            # Step 2: measurement model z = H x + noise
            # GPS measures position only, so H picks X and Y out of [X, Y, VX, VY]
            H = np.array([[1, 0, 0, 0],
                          [0, 1, 0, 0]])

            # Measurement noise covariance: independent X/Y, each with variance sigma_meas^2
            R = np.array([[GPS_POS_STD ** 2, 0],
                          [0, GPS_POS_STD ** 2]])

            # Measurement vector z_k (used in Step 3: y~ = z - H x^-)
            z = np.array([meas.x, meas.y])

            # Step 3: Kalman filter update equations
            # Innovation: measurement minus predicted measurement
            y = z - H @ state

            # Innovation covariance
            S = H @ cov @ H.T + R

            # Kalman gain
            K = cov @ H.T @ np.linalg.inv(S)

            # Updated state estimate
            state = state + K @ y

            # Updated covariance
            cov = (np.eye(4) - K @ H) @ cov

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
