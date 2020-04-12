import matplotlib.pyplot as plt
import numpy as np
import math


class Obstructions():
    def __init__(self, width, height, c):
        self.W = width + 1
        self.H = height + 1
        self.r = 2
        self.c = c
        self.x_obs = []
        self.y_obs = []
        self.showObstacle = False
        # self.map = np.zeros([self.H, self.W], dtype=np.int8)
        self.obstaclespace = []
        self.animationImage = np.zeros([self.H, self.W, 3])
        self.generateMap()
        # self.showMap()
        # self.r = 0
        # self.c = 0
        self.showObstacle = True
        self.generateMap()
        self.explored = np.zeros([self.H, self.W, 12], dtype=np.int8)  # self.obstaclespace.copy()
        self.parentData = np.zeros([self.H, self.W, 12, 6])
        self.dist_threshold = 1
        self.visited_matrix = np.zeros([int(102 / self.dist_threshold), int(102 / self.dist_threshold), 12])
        pass

    def generateMap(self):
        self.circle((51, 51), 10)
        self.circle((71, 81), 10)
        self.circle((31, 21), 10)
        self.circle((71, 21), 10)
        self.quad1()
        self.quad2()
        self.quad3()
        self.border()
        pass

    def circle(self, center, radius):
        center_x, center_y = center[0], center[1]
        for i in range(center_x - (radius), center_x + (radius)):
            for j in range(center_y - (radius), center_y + (radius)):
                if ((i - center_x) ** 2 + (j - center_y) ** 2) <= (radius) ** 2:
                    if not self.showObstacle:
                        self.obstaclespace.append([i, j])
                    else:
                        self.animationImage[102 - j, i] = np.array([255, 255, 255])

        return

    def check_circle(self, center, radius, i, j):
        center_x, center_y = center[0], center[1]
        if ((i - center_x) ** 2 + (j - center_y) ** 2) <= (radius + self.r + self.c) ** 2:
            return True
        else:
            return False

    def quad1(self):
        for i in range(0, 103):
            for j in range(0, 103):
                if (j - (88.5) <= 0) and (i - (38.5 ) <= 0) and \
                        (j - (73.5) >= 0) and (
                        i - 23.5 >= 0):
                    if not self.showObstacle:
                        self.obstaclespace.append([i, j])
                    else:
                        self.animationImage[102 - j, i] = np.array([255, 255, 255])

    def check_quad1(self, i, j):
        if (j - (88.5 + self.r + self.c) <= 0) and (i - (38.5 + self.r + self.c) <= 0) and \
                (j - (73.5 - (self.r + self.c)) >= 0) and (
                i - (23.5 - (self.r + self.c)) >= 0):
            return True
        else:
            return False

    def quad2(self):
        for i in range(0, 103):
            for j in range(0, 103):
                if (j - (58.5) <= 0) and (i - (18.5) <= 0) and \
                        (j - (43.5) >= 0) and (
                        i - (3.5) >= 0):
                    if not self.showObstacle:
                        self.obstaclespace.append([i, j])
                    else:
                        self.animationImage[102 - j, i] = np.array([255, 255, 255])

    def check_quad2(self, i, j):
        if (j - (58.5 + self.r + self.c) <= 0) and (i - (18.5 + self.r + self.c) <= 0) and \
                (j - (43.5 - (self.r + self.c)) >= 0) and (
                i - (3.5 - (self.r + self.c)) >= 0):
            return True
        else:
            return False

    def quad3(self):
        for i in range(0, 102):
            for j in range(0, 102):
                if (j - (58.5) <= 0) and (i - (98.5) <= 0) and \
                        (j - (43.5) >= 0) and (
                        i - (83.5) >= 0):
                    if not self.showObstacle:
                        self.obstaclespace.append([i, j])
                    else:
                        self.animationImage[102 - j, i] = np.array([255, 255, 255])

    def check_quad3(self, i, j):
        if (j - (58.5 + self.r + self.c) <= 0) and (i - (98.5 + self.r + self.c) <= 0) and \
                (j - (43.5 - (self.r + self.c)) >= 0) and (
                i - (83.5 - (self.r + self.c)) >= 0):
            return True
        else:
            return False

    def border(self):
        self.total = self.r + self.c
        for i in range(0, 102):
            for j in range(0, 102):
                if j >= 102 - self.total:
                    if not self.showObstacle:
                        self.obstaclespace.append([i, j])
                    else:
                        self.animationImage[j, i] = np.array([255, 255, 255])
        for i in range(0, 102):
            for j in range(0, 102):
                if i >= 102 - self.total:
                    if not self.showObstacle:
                        self.obstaclespace.append([i, j])
                    else:
                        self.animationImage[j, i] = np.array([255, 255, 255])
        for i in range(0, 102):
            for j in range(0, 102):
                if j <= self.total:
                    if not self.showObstacle:
                        self.obstaclespace.append([i, j])
                    else:
                        self.animationImage[j, i] = np.array([255, 255, 255])
        for i in range(0, 102):
            for j in range(0, 102):
                if i <= self.total:
                    if not self.showObstacle:
                        self.obstaclespace.append([i, j])
                    else:
                        self.animationImage[j, i] = np.array([255, 255, 255])


    def check_border(self, i, j):
        if j >= 102 - self.total:
            return True
        if i >= 102 - self.total:
            return True
        if j <= self.total:
            return True
        if i <= self.total:
            return True
        else:
            return False


    def showMap(self, ax):
        # plt.imshow(self.map, cmap="gray")

        obstaclespace = np.array(self.obstaclespace)
        self.x_obs = [col[0] for col in self.obstaclespace]
        self.y_obs = [col[1] for col in self.obstaclespace]

        ax.scatter(self.x_obs, self.y_obs, c='r')
        plt.imshow(self.obstaclespace, cmap="gray")
        # plt.axis([0, 300, 0, 200])
        plt.show()

    def checkallobstacle(self, i, j):
        if self.check_circle((51, 51), 10, i, j):
            return True
        elif self.check_circle((71, 81), 10, i, j):
            return True
        elif self.check_circle((31, 21), 10, i, j):
            return True
        elif self.check_circle((71, 21), 10, i, j):
            return True
        elif self.check_quad1(i, j):
            return True
        elif self.check_quad2(i,j):
            return True
        elif self.check_quad3(i, j):
            return True
        elif self.check_border(i, j):
            return True
        else:
            return False

    def checkFeasibility(self, node):
        h, w = node[0], node[1]
        if h >= self.H or w >= self.W or h < 0 or w < 0:
            return False
        ##elif [h, w] in self.obstaclespace:
        #    return False
        # else:
        #   return True
        elif self.checkallobstacle(w, h):
            return False
        else:
            return True

    def intersection(self, node1, node2, line, range1, range2):
        x, y = node1[0], node1[1]
        u, v = node2[0], node2[2]
        a1 = v - y
        b1 = u - x
        c1 = a1 * x + b1 * u
        a2, b2, c2 = line[0], line[1], line[2]
        A = np.array([[a1, b1], [a2, b2]])
        B = np.array([c1, c2])
        C = np.linalg.solve(A, B)
        p, q = C[0], C[1]
        if range1[0] < p < range1[1] and range2[0] < q < range2[1]:
            return False
        else:
            return True

    #  def checkVisited(self, node):
    #     if self.explored[node[0], node[1]] == 1:
    #         return False
    #     else:
    #         return True

    def checkVisited(self, node):
        if node[0] % 1 > self.dist_threshold / 2:
            i = int(math.ceil(node[0]) / self.dist_threshold)  # uppervalue
        else:
            i = int(math.floor(node[0]) / self.dist_threshold)

        if node[1] % 1 > self.dist_threshold / 2:  # lower value
            j = int(math.ceil(node[1]) / self.dist_threshold)
        else:
            j = int(math.floor(node[1]) / self.dist_threshold)
        i = 102 if i > 102 else i
        j = 102 if j > 102 else j
        m = node[1]
        ###for K, changed with 30 degrees.
        if m >= 360:
            m = m % 360
        if 15 < m <= 45 or -345 <= m < -315:
            k = 1
        elif 45 < m <= 75 or -315 <= m < -285:
            k = 2
        elif 75 < m <= 105 or -285 <= m < -255:
            k = 3
        elif 105 < m <= 135 or -255 <= m < -225:
            k = 4
        elif 135 < m <= 165 or -225 <= m < -195:
            k = 5
        elif 165 < m <= 195 or -195 <= m < -165:
            k = 6
        elif 195 < m <= 225 or -165 <= m < -135:
            k = 7
        elif 225 < m <= 255 or -135 <= m < -105:
            k = 8
        elif 255 < m <= 285 or -105 <= m < -75:
            k = 9
        elif 285 < m <= 315 or -75 <= m < -45:
            k = 10
        elif 315 < m <= 345 or -45 <= m < -15:
            k = 11
        elif 345 < m < 360 or -360 < m < -345 or -15 <= m <= 15:
            k = 0
        # print(i, j, k)
        if self.explored[i][j][k] == 1:
            return False
        else:
            return True
        # self.visited_matrix[i][j][k] = 1
        # return self.visited_matrix

    def addParent(self, node, parentNode, cost, action1, action2):
        if node[0] % 1 > self.dist_threshold / 2:
            i = int(math.ceil(node[0]) / self.dist_threshold)  # uppervalue
        else:
            i = int(math.floor(node[0]) / self.dist_threshold)

        if node[1] % 1 > self.dist_threshold / 2:  # lower value
            j = int(math.ceil(node[1]) / self.dist_threshold)
        else:
            j = int(math.floor(node[1]) / self.dist_threshold)

        i = 102 if i > 102 else i
        j = 102 if j > 102 else j
        m = node[1]
        ###for K, changed with 30 degrees.
        if m >= 360:
            m = m % 360
        if 15 < m <= 45 or -345 <= m < -315:
            k = 1
        elif 45 < m <= 75 or -315 <= m < -285:
            k = 2
        elif 75 < m <= 105 or -285 <= m < -255:
            k = 3
        elif 105 < m <= 135 or -255 <= m < -225:
            k = 4
        elif 135 < m <= 165 or -225 <= m < -195:
            k = 5
        elif 165 < m <= 195 or -195 <= m < -165:
            k = 6
        elif 195 < m <= 225 or -165 <= m < -135:
            k = 7
        elif 225 < m <= 255 or -135 <= m < -105:
            k = 8
        elif 255 < m <= 285 or -105 <= m < -75:
            k = 9
        elif 285 < m <= 315 or -75 <= m < -45:
            k = 10
        elif 315 < m <= 345 or -45 <= m < -15:
            k = 11
        elif 345 < m < 360 or -360 < m < -345 or -15 <= m <= 15:
            k = 0

        # self.parentData[i,j,k,:3] = np.array(parentNode)
        self.parentData[i, j, k, 0] = parentNode[0]
        self.parentData[i, j, k, 1] = parentNode[1]
        # print("node2", parentNode)
        self.parentData[i, j, k, 2] = parentNode[2]
        self.parentData[i, j, k, 3] = cost
        self.parentData[i, j, k, 4] = action1
        self.parentData[i, j, k, 5] = action2

        # pass

    def getParent(self, node):
        if node[0] % 1 > self.dist_threshold / 2:
            i = int(math.ceil(node[0]) / self.dist_threshold)  # uppervalue
        else:
            i = int(math.floor(node[0]) / self.dist_threshold)

        if node[1] % 1 > self.dist_threshold / 2:  # lower value
            j = int(math.ceil(node[1]) / self.dist_threshold)
        else:
            j = int(math.floor(node[1]) / self.dist_threshold)

        i = 102 if i > 102 else i
        j = 102 if j > 102 else j

        m = node[1]
        ###for K, changed with 30 degrees.
        if m >= 360:
            m = m % 360
        if 15 < m <= 45 or -345 <= m < -315:
            k = 1
        elif 45 < m <= 75 or -315 <= m < -285:
            k = 2
        elif 75 < m <= 105 or -285 <= m < -255:
            k = 3
        elif 105 < m <= 135 or -255 <= m < -225:
            k = 4
        elif 135 < m <= 165 or -225 <= m < -195:
            k = 5
        elif 165 < m <= 195 or -195 <= m < -165:
            k = 6
        elif 195 < m <= 225 or -165 <= m < -135:
            k = 7
        elif 225 < m <= 255 or -135 <= m < -105:
            k = 8
        elif 255 < m <= 285 or -105 <= m < -75:
            k = 9
        elif 285 < m <= 315 or -75 <= m < -45:
            k = 10
        elif 315 < m <= 345 or -45 <= m < -15:
            k = 11
        elif 345 < m < 360 or -360 < m < -345 or -15 <= m <= 15:
            k = 0

        return tuple([self.parentData[i, j, k, 0], self.parentData[i, j, k, 1], self.parentData[i, j, k, 2], self.parentData[i, j, k, 4], self.parentData[i, j, k, 5]])

    def getCost(self, node):
        if node[0] % 1 > self.dist_threshold / 2:
            i = int(math.ceil(node[0]) / self.dist_threshold)  # uppervalue
        else:
            i = int(math.floor(node[0]) / self.dist_threshold)

        if node[1] % 1 > self.dist_threshold / 2:  # lower value
            j = int(math.ceil(node[1]) / self.dist_threshold)
        else:
            j = int(math.floor(node[1]) / self.dist_threshold)

        i = 102 if i > 102 else i
        j = 102 if j > 102 else j
        m = node[1]
        ###for K, changed with 30 degrees.
        if m >= 360:
            m = m % 360
        if 15 < m <= 45 or -345 <= m < -315:
            k = 1
        elif 45 < m <= 75 or -315 <= m < -285:
            k = 2
        elif 75 < m <= 105 or -285 <= m < -255:
            k = 3
        elif 105 < m <= 135 or -255 <= m < -225:
            k = 4
        elif 135 < m <= 165 or -225 <= m < -195:
            k = 5
        elif 165 < m <= 195 or -195 <= m < -165:
            k = 6
        elif 195 < m <= 225 or -165 <= m < -135:
            k = 7
        elif 225 < m <= 255 or -135 <= m < -105:
            k = 8
        elif 255 < m <= 285 or -105 <= m < -75:
            k = 9
        elif 285 < m <= 315 or -75 <= m < -45:
            k = 10
        elif 315 < m <= 345 or -45 <= m < -15:
            k = 11
        elif 345 < m < 360 or -360 < m < -345 or -15 <= m <= 15:
            k = 0

        return self.parentData[i, j, k, 3]

    def addExplored(self, node):
        if node[0] % 1 > self.dist_threshold / 2:
            i = int(math.ceil(node[0]) / self.dist_threshold)  # uppervalue
        else:
            i = int(math.floor(node[0]) / self.dist_threshold)

        if node[1] % 1 > self.dist_threshold / 2:  # lower value
            j = int(math.ceil(node[1]) / self.dist_threshold)
        else:
            j = int(math.floor(node[1]) / self.dist_threshold)

        i = 102 if i > 102 else i
        j = 102 if j > 102 else j
        m = node[1]
        ###for K, changed with 30 degrees.
        if m >= 360:
            m = m % 360
        if 15 < m <= 45 or -345 <= m < -315:
            k = 1
        elif 45 < m <= 75 or -315 <= m < -285:
            k = 2
        elif 75 < m <= 105 or -285 <= m < -255:
            k = 3
        elif 105 < m <= 135 or -255 <= m < -225:
            k = 4
        elif 135 < m <= 165 or -225 <= m < -195:
            k = 5
        elif 165 < m <= 195 or -195 <= m < -165:
            k = 6
        elif 195 < m <= 225 or -165 <= m < -135:
            k = 7
        elif 225 < m <= 255 or -135 <= m < -105:
            k = 8
        elif 255 < m <= 285 or -105 <= m < -75:
            k = 9
        elif 285 < m <= 315 or -75 <= m < -45:
            k = 10
        elif 315 < m <= 345 or -45 <= m < -15:
            k = 11
        elif 345 < m < 360 or -360 < m < -345 or -15 <= m <= 15:
            k = 0
        # print("aaaa", i, j, k)
        self.explored[i][j][k] = 1
        self.animationImage[102 - int(node[0]), int(node[1]), :] = np.array([0, 0, 255])

    def path(self, d):
        # for d in data:

        self.animationImage[102 - int(d[0]), int(d[1]), :] = np.array([0, 255, 0])
        # print("showing final")
        # plt.imshow(self.animationImage)
        # plt.show()
        return
