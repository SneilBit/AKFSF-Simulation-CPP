# Python equivalent of kalmanfilter.h
#
# Eigen -> numpy notes:
#   VectorXd / Vector2d / Vector4d   -> 1-D np.ndarray
#   MatrixXd / Matrix2d / Matrix4d   -> 2-D np.ndarray
#   A * B, A.transpose()             -> A @ B, A.T
#   cov.llt().matrixL()              -> llt_matrixL(cov)
#   S.inverse()                      -> inverse(S)

import numpy as np

from car import VehicleState
from sensors import GPSMeasurement, GyroMeasurement, LidarMeasurement
from beacons import BeaconData, BeaconMap


def llt_matrixL(matrix):
    """Eigen::LLT<MatrixXd>(matrix).matrixL()

    Reproduces Eigen's unblocked in-place Cholesky (used for matrices smaller than 32x32),
    including its behaviour on non positive-definite input: it stops at the failing column
    instead of raising like np.linalg.cholesky does.
    """
    mat = np.array(matrix, dtype=float, copy=True)
    size = mat.shape[0]
    for k in range(size):
        rs = size - k - 1  # remaining size
        x = mat[k, k]
        if k > 0:
            x -= np.dot(mat[k, :k], mat[k, :k])
        if x <= 0.0:
            break
        x = np.sqrt(x)
        mat[k, k] = x
        if k > 0 and rs > 0:
            mat[k + 1:, k] -= mat[k + 1:, :k] @ mat[k, :k]
        if rs > 0:
            mat[k + 1:, k] /= x
    return np.tril(mat)


def inverse(matrix):
    """MatrixXd::inverse() - Eigen does not throw on a singular matrix, so neither does this."""
    try:
        return np.linalg.inv(matrix)
    except np.linalg.LinAlgError:
        return np.full(np.shape(matrix), np.nan)


class KalmanFilterBase:

    def __init__(self):
        self.__m_initialised = False
        self.__m_state = np.zeros(0)
        self.__m_covariance = np.zeros((0, 0))

    def reset(self):
        self.__m_initialised = False

    def isInitialised(self):
        return self.__m_initialised

    # protected:

    def getState(self):
        return np.array(self.__m_state, dtype=float, copy=True)

    def getCovariance(self):
        return np.array(self.__m_covariance, dtype=float, copy=True)

    def setState(self, state):
        self.__m_state = np.array(state, dtype=float, copy=True)
        self.__m_initialised = True

    def setCovariance(self, cov):
        self.__m_covariance = np.array(cov, dtype=float, copy=True)

    # C++ overloads of KalmanFilter::predictionStep():
    #   predictionStep(double dt)                      -> predictionStepDt(dt)
    #   predictionStep(GyroMeasurement gyro, double dt) -> predictionStepGyro(gyro, dt)
    def predictionStep(self, *args):
        if len(args) == 1:
            return self.predictionStepDt(args[0])
        return self.predictionStepGyro(args[0], args[1])


# class KalmanFilter : public KalmanFilterBase
#     VehicleState getVehicleState();
#     Matrix2d getVehicleStatePositionCovariance();
#     void predictionStepDt(double dt);
#     void predictionStepGyro(GyroMeasurement gyro, double dt);
#     void handleLidarMeasurements(const std::vector<LidarMeasurement>& meas, const BeaconMap& map);
#     void handleLidarMeasurement(LidarMeasurement meas, const BeaconMap& map);
#     void handleGPSMeasurement(GPSMeasurement meas);
#
# is implemented in kalmanfilter.py (copy one of the kalmanfilter_*.py files over it).
