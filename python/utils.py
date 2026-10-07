import math

import numpy as np

from display import Vector2, offsetPoints


def wrapAngle(angle):
    angle = math.fmod(angle, (2.0 * math.pi))
    if angle <= -math.pi:
        angle += (2.0 * math.pi)
    elif angle > math.pi:
        angle -= (2.0 * math.pi)
    return angle


def calculateMean(dataset):
    if len(dataset) == 0:
        return math.nan
    total = sum(dataset, 0.0)
    mean = total / len(dataset)
    return mean


def calculateRMSE(dataset):
    accum = 0.0
    if len(dataset) == 0:
        return 0.0
    for d in dataset:
        accum += (d * d)
    rmse = math.sqrt(accum / (len(dataset)))
    return rmse


def generateEllipse(x, y, sigma_xx, sigma_yy, sigma_xy, num_points=50):
    pos_cov = np.array([[sigma_xx, sigma_xy], [sigma_xy, sigma_yy]], dtype=float)

    if not np.all(np.isfinite(pos_cov)):
        # Eigen's JacobiSVD propagates NaN/Inf instead of raising like numpy does
        return [Vector2(math.nan, math.nan) for _ in range(num_points)]

    U, singularValues, _ = np.linalg.svd(pos_cov)
    D = U @ np.diag(3.0 * np.sqrt(singularValues))

    theta = np.linspace(0, 2 * math.pi, num_points)
    theta_array = np.vstack((np.cos(theta), np.sin(theta)))
    A = D @ theta_array

    shape_body = []
    for i in range(A.shape[1]):
        shape_body.append(Vector2(A[0, i], A[1, i]))
    shape_world = offsetPoints(shape_body, Vector2(x, y))

    return shape_world


def generateCircle(x, y, radius, num_points=50):
    theta = np.linspace(0, 2 * math.pi, num_points)
    circle = []
    for i in range(theta.size):
        circle.append(Vector2(x + radius * math.cos(theta[i]), y + radius * math.sin(theta[i])))
    return circle
