import random
import numpy as np
import math
import Queue as q
from sklearn.neighbors import NearestNeighbors
import Roadmap1
import time
import ast
import rospy
from pqp_server.srv import *

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


def pqp_client(q, q_prime):
    a = random.random()
    if a < 0.9:
        return False
    else:
        return True

class PRM:

    def __init__(self, roadmap, start, goal):
        self.V = roadmap.V
        self.E = roadmap.E
        self.start = start
        self.goal = goal
        self.dist = {}  # distance dictionary
        self.Q = q.PriorityQueue()
        self.prev = {}  # find parents
        self.rout = []


    def add_start_goal_to_graph(self):
        self.V.append(self.start)
        self.V.append(self.goal)
        num = NUM_NN
        N_indices = self.find_nearest_neighbor(self.start, num)
        while True:
            num_old = len(self.E)
            for i in range(len(N_indices) - 1):
                indice = N_indices[i + 1]
                q_prime = self.V[indice]
                if self.if_path_collisionfree(self.start, q_prime):
                    self.E.append((self.start, q_prime))
            num_new = len(self.E)
            if num_old != num_new:
                break
            num = num + NUM_NN

        num = NUM_NN
        N_indices = self.find_nearest_neighbor(self.goal, num)
        while True:
            num_old = len(self.E)
            for i in range(len(N_indices) - 1):
                indice = N_indices[i + 1]
                q_prime = self.V[indice]
                if self.if_path_collisionfree(self.goal, q_prime):
                    self.E.append((self.goal, q_prime))
            num_new = len(self.E)
            if num_old != num_new:
                break
            num = num + NUM_NN

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

    def distance(self, q0, q1):
        w0 = 1
        w1 = 3
        # t_dis = 0  # translation distance
        # r_dis = 0  # rotation distance
        t0 = q0[0]
        t1 = q1[0]
        r0 = np.asarray(q0[1])
        r1 = np.asarray(q1[1])
        t_dis = (t0[0]-t1[0])**2 + (t0[1]-t1[1])**2 + (t0[2]-t1[2])**2
        lam = np.inner(r0, r1)
        r_dis = 1 - np.linalg.norm(lam)
        distance = w0 * t_dis + w1 * r_dis
        return distance

    def find_succ(self, q):
        succ = []
        #print('q', (tuple(self.E[0][0]), tuple(self.E[0][0])))
        for i in range(len(self.E)):
            if q == (tuple(self.E[i][0][0]), tuple(self.E[i][0][1])):
                succ.append(self.E[i][1])
            elif q == (tuple(self.E[i][1][0]), tuple(self.E[i][1][1])):
                succ.append(self.E[i][0])
        return succ

    def find_rout(self):

        #print(tuple(self.start))
        self.dist[(tuple(self.start[0]), tuple(self.start[1]))] = 0
        for i in range(len(self.V)):
            if self.V[i] != self.start:
                self.dist[(tuple(self.V[i][0]), tuple(self.V[i][1]))] = 1000000
            # self.prev[(tuple(self.V[i][0]), tuple(self.V[i][1]))] =
            self.Q.put((self.dist[(tuple(self.V[i][0]), tuple(self.V[i][1]))], (tuple(self.V[i][0]), tuple(self.V[i][1]))))

        while not self.Q.empty():
            u = self.Q.get()[1]
            #print('1',u)
            succ = self.find_succ(u)
            #print('1', succ)
            for i in range(len(succ)):
                v = succ[i]
                alt = self.dist[(tuple(u[0]), tuple(u[1]))] + self.distance(u, v)
                if alt < self.dist[(tuple(v[0]), tuple(v[1]))]:
                    self.dist[(tuple(v[0]), tuple(v[1]))] = alt
                    self.prev[(tuple(v[0]), tuple(v[1]))] = u
                    self.Q.put((alt, (tuple(v[0]), tuple(v[1]))))

    def print_rout(self):
        self.rout = []
        self.rout.insert(0, self.goal)
        next_ = self.prev[(tuple(self.rout[0][0]), tuple(self.rout[0][1]))]
        while next_ != (tuple(self.start[0]), tuple(self.start[1])):
            self.rout.insert(0, next_)
            next_ = self.prev[(tuple(self.rout[0][0]), tuple(self.rout[0][1]))]
        self.rout.insert(0, next_)
        print('rout', self.rout)

    def print_solution_quality(self):
        distance = 0
        for i in range(len(self.rout) - 1):
            distance += self.distance(self.rout[i], self.rout[i+1])
        quality = 1 / distance
        print('path quality:', quality)
        return quality



        # print(self.prev[(tuple(self.goal[0]), tuple(self.goal[1]))])


if __name__ == '__main__':
    start_list = []
    goal_list = []
    time_list = []
    quality_list = []

    # open sample file
    f1 = open('source/start_list.txt')
    temp = []
    for line in f1:
        # print(line)
        temp.append(line)
    #
    for i in range(len(temp)):
        start_list.append(ast.literal_eval(temp[i]))

    f2 = open('source/goal_list.txt')
    temp = []
    for line in f2:
        # print(line)
        temp.append(line)
    #
    for i in range(len(temp)):
        goal_list.append(ast.literal_eval(temp[i]))



    roadmap1 = Roadmap1.RoadMap()
    roadmap1.build_roadmap()
    print('roadmap_size', len(roadmap1.E))


    for i in range(len(start_list)):
        start_time = time.time()
        prm = PRM(roadmap1, start_list[i], goal_list[i])
        prm.add_start_goal_to_graph()
        prm.find_rout()
        prm.print_rout()
        quality_list.append(prm.print_solution_quality())
        end_time = time.time()
        time_used = end_time - start_time
        time_list.append(time_used)

        print('time used', time_used)

    print('time list', time_list)
    print('quality list', quality_list)

    with open('time_list_1.txt', 'w') as f:
        for i in range(len(time_list)):
            if i != len(time_list) - 1:
                f.write(str(time_list[i]) + '\n')
            else:
                f.write(str(time_list[i]))

    with open('quality_list_1.txt', 'w') as f:
        for i in range(len(quality_list)):
            if i != len(quality_list) - 1:
                f.write(str(quality_list[i]) + '\n')
            else:
                f.write(str(quality_list[i]) + '\n')


