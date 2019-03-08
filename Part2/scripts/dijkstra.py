import ast
import Queue as q
import math

'''f1 = open('source/adj1.txt')
index_list = []
temp = []
for line in f1:
    # print(line)
    temp.append(line)
#
for i in range(len(temp)):
    index_list.append(ast.literal_eval(temp[i]))


f2 = open('source/coor1.txt')
coor_list = []
temp = []
for line in f2:
    # print(line)
    temp.append(line)
#
for i in range(len(temp)):
    coor_list.append(ast.literal_eval(temp[i]))'''


class Dijkstra:
    def __init__(self, index_list, coor_list):
        self.E = index_list
        self.coor = coor_list
        self.start = 0
        self.goal = len(self.coor) - 1
        self.dist = {}  # distance dictionary
        self.Q = q.PriorityQueue()
        self.prev = {}  # find parents
        self.rout = []

    def find_succ(self, u):
        succ = []
        #print('q', (tuple(self.E[0][0]), tuple(self.E[0][0])))
        for i in range(len(self.E)):
            if u == self.E[i][0]:
                succ.append(self.E[i][1])
            elif u == self.E[i][1]:
                succ.append(self.E[i][0])
        return succ

    def distance(self, u, v):
        coor_u = self.coor[u]
        coor_v = self.coor[v]
        dist = abs(coor_u[0] - coor_v[0]) + abs(coor_u[1] - coor_v[1])
        return dist

    def find_rout(self):
        self.dist[self.start] = 0
        for i in range(len(self.coor)):
            if i != self.start:
                self.dist[i] = 1000000
            self.Q.put((self.dist[i], i))
        while not self.Q.empty():
            u = self.Q.get()[1]
            succ = self.find_succ(u)
            for i in succ:
                v = i
                alt = self.dist[u] + self.distance(u, v)
                if alt < self.dist[v]:
                    self.dist[v] = alt
                    self.prev[v] = u
                    self.Q.put((alt, v))

    def print_rout(self):
        self.rout.insert(0, self.goal)
        next_ = self.prev[self.rout[0]]
        while next_ != self.start:
            self.rout.insert(0, next_)
            next_ = self.prev[self.rout[0]]
        self.rout.insert(0, next_)
        print('rout', self.rout)

    def print_solution_quality(self):
        distance = 0
        for i in range(len(self.rout) - 1):
            distance += self.distance(self.rout[i], self.rout[i+1])
        quality = 1/distance
        print('path quality:', quality)
        return quality



'''if __name__ == '__main__':
    e_end = [len(coor_list)-1, len(coor_list)]  # append last edge connect goal to the end point
    index_list.append(e_end)
    coor_list.append([9, 5.5])  # the coordinate of the goal

    dij = Dijkstra(index_list, coor_list)
    dij.find_rout()
    dij.print_rout()
    dij.print_solution_quality()'''
