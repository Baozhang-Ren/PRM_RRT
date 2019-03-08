import random
import numpy as np
import math
import queue as q
from sklearn.neighbors import NearestNeighbors
import Roadmap2
import time

NUM_SAMPLE = 500  # number of samples
NUM_NN = 10  # Number of nearest neighbor

X_MAX = 10
X_MIN = -10
Y_MAX = 10
Y_MIN = -10
Z_MAX = 3
Z_MIN = 0

def sample_config():
    T = [0.0, 0.0, 0.0]
    R = [0.0, 0.0, 0.0, 0.0]

    T[0] = (X_MAX - X_MIN) * random.random() + X_MIN
    T[1] = (Y_MAX - Y_MIN) * random.random() + Y_MIN
    T[2] = 0

    s = random.random()
    sigma1 = 0
    sigma2 = 1
    theta1 = 2 * math.pi * random.random()
    theta2 = 2 * math.pi * random.random()
    w = math.cos(theta2) * sigma2
    x = math.sin(theta1) * sigma1
    y = math.cos(theta1) * sigma1
    z = math.sin(theta2) * sigma2
    R[0] = w
    R[1] = x
    R[2] = y
    R[3] = z
    return tuple((T, R))

def quaternion_to_rotation_matrix(quaternion):
    w = quaternion[0]
    x = quaternion[1]
    y = quaternion[2]
    z = quaternion[3]
    M = [0] * 9
    M[0] = 1 - 2 * (y ** 2) - 2 * (z ** 2)
    M[1] = 2 * x * y + 2 * w * z
    M[2] = 2 * x * z - 2 * w * y
    M[3] = 2 * x * y - 2 * w * z
    M[4] = 1 - 2 * (x ** 2) - 2 * (z ** 2)
    M[5] = 2 * y * z + 2 * w * x
    M[6] = 2 * x * z + 2 * w * y
    M[7] = 2 * y * z - 2 * w * x
    M[8] = 1 - 2 * (x ** 2) - 2 * (y ** 2)
    return M


def pqp_client(q, q_prime):
    a = random.random()
    if a < 0.9:
        return False
    else:
        return True


if __name__ == '__main__':
    start_list = []
    goal_list = []

    for i in range(50):
        # sample start and goal
        while True:
            start = sample_config()
            result = pqp_client(start[0], quaternion_to_rotation_matrix(start[1]))
            if result == False:
                break
        start_list.append(start)

        while True:
            goal = sample_config()
            result = pqp_client(goal[0], quaternion_to_rotation_matrix(goal[1]))
            if result == False:
                break
        goal_list.append(goal)

    print(len(start_list))
    print(len(goal_list))

    with open('start_list.txt', 'w') as f:
        for i in range(len(start_list)):
            if i != len(start_list) - 1:
                f.write(str(start_list[i]) + '\n')
            else:
                f.write(str(start_list[i]))

    with open('goal_list.txt', 'w') as f:
        for i in range(len(goal_list)):
            if i != len(goal_list) - 1:
                f.write(str(goal_list[i]) + '\n')
            else:
                f.write(str(goal_list[i]) + '\n')