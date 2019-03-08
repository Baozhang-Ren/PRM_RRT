import random
import numpy as np
import math
from sklearn.neighbors import NearestNeighbors


NUM_SAMPLE = 500  # number of samples
NUM_NN = math.ceil(math.exp(1) * (1 + 1/3) * math.log(NUM_SAMPLE, 2))  # Number of nearest neighbor

X_MAX = 10
X_MIN = -10
Y_MAX = 10
Y_MIN = -10
Z_MAX = 3
Z_MIN = 0

# for i in range(100):
#     ran = random.random()
#     print(ran)


def sample_config():
    T = [0.0, 0.0, 0.0]
    R = [0.0, 0.0, 0.0, 0.0]

    T[0] = (X_MAX - X_MIN) * random.random() + X_MIN
    T[1] = (Y_MAX - Y_MIN) * random.random() + Y_MIN
    T[2] = (Z_MAX - Z_MIN) * random.random() + Z_MIN

    s = random.random()
    sigma1 = math.sqrt(1 - s)
    sigma2 = math.sqrt(s)
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


def pqp_client(T, R):
    a = random.random()
    if a < 0.9:
        return False
    else:
        return True


class RoadMap:
    """
    Build Roadmap
    """
    def __init__(self):
        self.V = []  # reserve configurations
        self.E = []  # path (q, q_prime)

    def distance_angle(self, theta0, theta1):
        theta_prime = theta1 - theta0
        if theta_prime < - math.pi:
            theta_prime = theta_prime + 2 * math.pi
        elif theta_prime < math.pi:
            theta_prime = theta_prime - 2 * math.pi
        return theta_prime

    def distance(self, q0, q1):
        w0 = 1
        w1 = 1
        t_dis = 0  # translation distance
        r_dis = 0  # rotation distance
        t0 = q0[0]
        t1 = q1[0]
        r0 = q0[1]
        r1 = q1[1]
        t_dis = (t0[0]-t1[0])**2 + (t0[1]-t1[1])**2 + (t0[2]-t1[2])**2
        r_dis = math.sqrt(self.distance_angle(r0[0], r1[0])**2 + self.distance_angle(r0[1], r1[1])**2 + self.distance_angle(r0[2], r1[2])**2)
        distance = w0 * t_dis + w1 * r_dis
        return distance

    def find_nearest_neighbor(self, q, numnn):
        num_nn = numnn
        if len(self.V) < num_nn + 1:
            n_neighbor = len(self.V)
        else:
            n_neighbor = num_nn + 1

        q = list(q)
        q = [q[0] + q[1]]
        q = np.asarray(q)
        V = list(self.V)
        for i in range(len(V)):
            V[i] = list(V[i])
            V[i] = V[i][0] + V[i][1]
            V[i] = np.asarray(V[i])
        nbrs = NearestNeighbors(n_neighbors=n_neighbor, algorithm='ball_tree').fit(V)
        distances, indices = nbrs.kneighbors(q)
        return indices[0]

    def if_path_collisionfree(self, q, q_prime):
        # t = q[0]
        # r = q[1]
        # t_prime = q_prime[0]
        # r_prime = q_prime[1]


        q = np.asarray(q)
        q[0] = np.asarray(q[0])
        q[1] = np.asarray(q[1])


        q_prime = np.asarray(q_prime)
        q_prime[0] = np.asarray(q_prime[0])
        q_prime[1] = np.asarray(q_prime[1])

        n_step = 10
        dq = (q_prime - q)/n_step

        for i in range(n_step):
            q = q + dq
            if pqp_client(q[0], quaternion_to_rotation_matrix(q[1])) == True:
                return False
        return True

    def build_roadmap(self):
        for i in range(NUM_SAMPLE):
            while True:
                q = sample_config()
                result = pqp_client(q[0], quaternion_to_rotation_matrix(q[1]))
                if result == False:
                    break
            self.V.append(q)    # add configuration q to V
            N_indices = self.find_nearest_neighbor(q, NUM_NN)
            for i in range(len(N_indices)-1):
                indice = N_indices[i+1]
                q_prime = self.V[indice]
                if self.if_path_collisionfree(q, q_prime):
                    self.E.append((q, q_prime))


if __name__ == '__main__':
    print(NUM_NN)
    roadmap = RoadMap()
    roadmap.build_roadmap()
    print(len(roadmap.E))



