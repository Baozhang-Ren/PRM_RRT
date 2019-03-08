from sklearn.neighbors import NearestNeighbors
import numpy as np
import math
import random

X_MAX = 10
X_MIN = -10
Y_MAX = 10
Y_MIN = -10
Z_MAX = 5
Z_MIN = 0

def sample_config():
    T = [0.0, 0.0, 0.0]
    R = [0.0, 0.0, 0.0]

    T[0] = (X_MAX - X_MIN) * random.random() + X_MIN
    T[1] = (Y_MAX - Y_MIN) * random.random() + Y_MIN
    T[2] = (Z_MAX - Z_MIN) * random.random() + Z_MIN

    R[0] = 2 * math.pi * random.random() - math.pi  # theta
    R[1] = math.acos(1 - 2 * random.random()) + math.pi/2  # phi
    if random.random() < (1 / 2):
        if R[1] < math.pi:
            R[1] = R[1] + math.pi
        else:
            R[1] = R[1] - math.pi
    R[2] = 2 * math.pi * random.random() - math.pi  # eta
    return tuple((T, R))





X = np.array([[-1, -1], [-2, -1], [-3, -2], [1, 1], [2, 1], [3, 2]])
nbrs = NearestNeighbors(n_neighbors=5, algorithm='ball_tree').fit(X)
distances, indices = nbrs.kneighbors([[-2, -1]])
print(indices)
