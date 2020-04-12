import sys

sys.path.remove(sys.path[1])
import cv2
from obstacle import Obstructions
import time
import argparse
from heapq import heappush, heappop
import math
import matplotlib.pyplot as plt


class PathFinder():
    def __init__(self, start, end, clearance, rpm1, rpm2):
        self.start = (start[1], start[0], start[2])
        self.end = (end[1], end[0])
        self.rpm1 = rpm1
        self.rpm2 = rpm2
        self.allNodes = []
        ############### nodeID [0], pareent [1], node [2], cost [3]  ## For BFS
        # self.mainData = [[0, 0, self.start, 0]]
        ############### cost , node  #### For Dijkstar
        self.Data = []
        self.allPose = [self.start]
        self.actionSet(self.start)
        self.possibleMove = len(self.actionset)
        self.temp = []
        self.obss = Obstructions(102, 102, clearance)

        self.goalReach = False
        self.view = True
        self.finalGoalState = []
        self.trace = []
        self.showCounter = 0
        self.skipFrame = 1
        self.output = []


    def plot_arrow(self, node, parentnode, color):
        Xi, Yi = parentnode[1], parentnode[0]
        UL, UR = node[3], node[4] #node[1], node[0]
        Thetai = parentnode[2]
        t = 0
        r = 0.15
        L = 3.54
        dt = 0.1
        Xn = Xi
        Yn = Yi
        Thetan = 3.14 * Thetai / 180

        while t < 1:
            t = t + dt
            Xs = Xn
            Ys = Yn
            Xn += 0.5 * r * (UL + UR) * math.cos(Thetan) * dt
            Yn += 0.5 * r * (UL + UR) * math.sin(Thetan) * dt
            Thetan += (r / L) * (UR - UL) * dt
            plt.plot([Xs, Xn], [Ys, Yn], color=color)

        Thetan = 180 * (Thetan) / 3.14

        # print(Xn, Yn, Thetan)
        return Xn, Yn, Thetan


    def initialCheck(self):
        if not self.obss.checkFeasibility(self.start):
            print("Start node is in obstacle field. Please provide correct starting position.")
            return False
        elif not self.obss.checkFeasibility(self.end):
            print("Goal node is in obstacle field. Please provide correct goal position.")
            return False
        else:
            return True


    def callforaction(self, Xi, Yi, Thetai, UL, UR):
        t = 0
        r = 0.38
        L = 3.54
        dt = 0.1
        Xn = Xi
        Yn = Yi
        Thetan = 3.14 * Thetai / 180

        while t < 1:
            t = t + dt
            Xs = Xn
            Ys = Yn
            Xn += 0.5 * r * (UL + UR) * math.cos(Thetan) * dt
            Yn += 0.5 * r * (UL + UR) * math.sin(Thetan) * dt
            Thetan += (r / L) * (UR - UL) * dt
            # print(Thetan, "first")
            # plt.plot([Xs, Xn], [Ys, Yn], color="blue")

        Thetan = (180 * (Thetan) / 3.14)
        # print(Thetan, "second")
        # print(Xn, Yn, Thetan)
        return Xn, Yn, Thetan, UL, UR


    def actionSet(self, node):

        x1, y1, th1, ac11, ac12 = self.callforaction(node[0], node[1], node[2], 0, self.rpm1)
        x2, y2, th2, ac21, ac22 = self.callforaction(node[0], node[1], node[2], self.rpm1, 0)
        x3, y3, th3, ac31, ac32 = self.callforaction(node[0], node[1], node[2], self.rpm1, self.rpm1)
        x4, y4, th4, ac41, ac42 = self.callforaction(node[0], node[1], node[2], 0, self.rpm2)
        x5, y5, th5, ac51, ac52 = self.callforaction(node[0], node[1], node[2], self.rpm2, 0)
        x6, y6, th6, ac61, ac62 = self.callforaction(node[0], node[1], node[2], self.rpm2, self.rpm2)
        x7, y7, th7, ac71, ac72 = self.callforaction(node[0], node[1], node[2], self.rpm1, self.rpm2)
        x8, y8, th8, ac81, ac82 = self.callforaction(node[0], node[1], node[2], self.rpm2, self.rpm1)

        self.actionset = ([round(x1, 0), round(y1, 0), th1, ac11, ac12],  # straight

                          [round(x2, 0), round(y2, 0), th2, ac21, ac22],  # moveup1

                          [round(x3, 0), round(y3, 0), th3, ac31, ac32],  # moveup2

                          [round(x4, 0), round(y4, 0), th4, ac41, ac42],  # movedown1

                          [round(x5, 0), round(y5, 0), th5, ac51, ac52],  # movedown2

                          [round(x6, 0), round(y6, 0), th6, ac61, ac62],

                          [round(x7, 0), round(y7, 0), th7, ac71, ac72],

                          [round(x8, 0), round(y8, 0), th8, ac81, ac82])

        # print("actionset", self.actionset)
        pass

    def checkEnd(self, currentNode):
        return (((currentNode[0] - self.end[0]) ** 2) + ((currentNode[1] - self.end[1]) ** 2)) <= 2.25

    def findNewPose(self, nodeState, action):
        tmp = nodeState[2]
        tmp = (tmp[0] + action[0], tmp[1] + action[1])
        return tmp

    def dijNewPose(self, node, action):
        tmp = (node[0] + action[0], node[1] + action[1], node[2] + action[2])
        return tmp

    def viewer(self, num):
        self.showCounter += 1
        if self.showCounter % num == 0:
            #cv2.namedWindow("Solver", cv2.WINDOW_NORMAL)
            #cv2.resizeWindow("Solver", 600, 600)
            #cv2.imshow("Solver", self.obss.animationImage)
            # plt.show(self.obss.obstaclespace)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.view = False
        pass

    def trackBack(self, node, ax):

        path = [self.end]
        child = tuple(node)
        tmp = self.end
        # print("child", tuple(child), self.start)
        count = 0
        while True: #child[0] != self.start[0] and child[1] != self.start[1]:

            self.obss.path(tmp)
            parent = self.obss.getParent(child)
            self.output.append([int(parent[3]), int(parent[4])])
            # print("temp",temp)
            self.plot_arrow(child, parent, 'b')
            # self.obss.showMap(ax)
            self.x_obs = [col[0] for col in self.obss.obstaclespace]
            self.y_obs = [col[1] for col in self.obss.obstaclespace]

            # ax.scatter(self.x_obs, self.y_obs, c='r')
            # filename = 'fig' + str(count) + '.png'
            count += 1
            # plt.savefig(filename, bbox_inches='tight')
            tmp = (int(child[0]), int(child[1]))

            # print(tmp)
            path.append(tmp)

            # print('parent', parent)
            child = parent
            if child[0] == self.start[0] and child[1] == self.start[1]:
                break
            self.viewer(1)
        #print(child, "child")
        #print(self.start, "start")
        print("out",self.output)
        return

    def DijkstraSolve(self):
        fig, ax = plt.subplots()
        plt.grid()
        ax.set_aspect('equal')
        plt.xlim(0, 102)
        plt.ylim(0, 102)
        plt.title('Final Output', fontsize=10)
        # plt.savefig('final_output.png', bbox_inches='tight')

        heappush(self.Data, (0, self.start, 0))
        # self.obss.astar(start, end)
        con = 0
        while len(self.Data) > 0:
            cost, node, cost_to_come = heappop(self.Data)
            # print("cost", cost)
            if self.checkEnd(node):
                self.goalReach = True
                print("goal reached")
                self.trackBack(node, ax)
                break
            # return

            self.actionSet(node)
            for action in self.actionset:
                # self.dg = math.sqrt(abs(((action[0] - end[0]) ** 2) + ((action[1] - end[1]) ** 2)))
                newPose = action
                act1, act2 = newPose[3], newPose[4]
                # self.vect(newPose, fig, ax,newPose)
                # print("newpose",newPose)
                if self.obss.checkFeasibility(newPose):
                    dg = math.sqrt(((newPose[0] - self.end[0]) ** 2) + ((newPose[1] - self.end[1]) ** 2))
                    newCost = cost_to_come + ((newPose[3] + newPose[4]) * 0.38 / 2)

                    # print("dg",self.dg)

                    if self.obss.checkVisited(newPose):
                        self.obss.addExplored(newPose)
                        self.plot_arrow(newPose, node, 'g')
                        # self.obss.showMap(ax)
                        self.x_obs = [col[0] for col in self.obss.obstaclespace]
                        self.y_obs = [col[1] for col in self.obss.obstaclespace]
                        # ax.scatter(self.x_obs, self.y_obs, c='r')
                        # filename = 'figs' + str(con) + '.png'
                        con += 1
                        # plt.savefig(filename, bbox_inches='tight')
                        # print(act1, act2, "act")
                        self.obss.addParent(newPose, node, newCost, act1, act2)
                        heappush(self.Data, (newCost + dg, newPose, newCost))

                    else:
                        if self.obss.getCost(newPose) > newCost + dg:
                            # self.obss.addExplored(newPose)
                            self.obss.addParent(newPose, node, newCost, act1, act2)
                    if self.view:
                        pass
                        self.viewer(30)
        if self.goalReach:
            self.obss.showMap(ax)
            # fig.show()
            # plt.draw()
            # plt.show()
        else:
            print("Could not find goal node...Leaving..!!")

        return


Parser = argparse.ArgumentParser()
#Parser.add_argument('--Start', default="[10,10,45]", help='Give inital point')
#Parser.add_argument('--End', default="[90, 90]", help='Give inital point')
#Parser.add_argument('--RobotRadius', default=3.54, help='Give inital point')
#Parser.add_argument('--Clearance', default=1, help='Give inital point')
Parser.add_argument('--ShowAnimation', default=1, help='1 if want to show animation else 0')
Parser.add_argument('--Framerate', default=30, help='Will show next step after this many steps. Made for fast viewing')
Args = Parser.parse_args()

#start = Args.Start
#end = Args.End
#r = int(Args.RobotRadius)
#c = int(Args.Clearance)
#u = [[5,10],[10,10],[10,10],[10,10],[10,10],[10,10],[10,10],[5,5],[5,5],[5,5],[10,5],[0,5],[5,5],[5,5],[5,0],[10,10],[0,5],[5,5],[5,0],[10,10],[10,10],[10,10],[10,10],[0,5],[5,5],[5,5],[5,0],[5,10],[5,5],[5,5],[10,10],[5,5]]
#v =[[0,5],[5,5],[5,5],[5,5],[5,0],[5,10],[5,10],[10,10],[5,0],[5,5],[5,5],[5,5],[5,0],[10,10],[10,10],[5,5],[10,5],[5,5],[5,10],[5,5],[10,5],[10,10],[10,10],[10,10]]
#start = [int(i) for i in start[1:-1].split(',')]
# start[1] = 200- start[1]
#end = [int(i) for i in end[1:-1].split(',')]
# end[1] = 200 - end[1]

start0 = int(input("Enter X coordinate of starting point: "))
start1 = int(input("Enter Y coordinate of starting point: "))
start2 = int(input("Enter orientation of robot in degrees: "))
end0 = int(input("Enter X coordinate of goal point: "))
end1 = int(input("Enter Y coordinate of goal point: "))
c = int(input("Enter Robot clearance: "))
rpm1 = int(input("Enter RPM1: "))
rpm2 = int(input("Enter RPM2: "))


start = [((start0 + 5) * 10), ((start1 + 5) * 10), start2]
end = [((end0 + 5) * 10), ((end1 + 5) * 10)]

v1 = (rpm1*((2*math.pi)/60))
v2 = (rpm2*((2*math.pi)/60))

#startp = [start0, start1, start2]
#endp = [end0, end1]

solver = PathFinder(start, end, c, v1, v2)
solver.view = int(Args.ShowAnimation)
solver.skipFrame = int(Args.Framerate)

if solver.initialCheck():
    startTime = time.time()
    solver.DijkstraSolve()
    print(time.time() - startTime)
    print("Press (q) to exit")
    solver.viewer(1)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    pass

################################################################################
try:
    import sim 
except:
    print('--------------------------------------------------------------')
    print('"sim.py" could not be imported. This means very probably that')
    print('either "sim.py" or the remoteApi library could not be found.')
    print('Make sure both are in the same folder as this file,')
    print('or appropriately adjust the file "sim.py"')
    print('--------------------------------------------------------------')
    print('')

import time

print('Program started')
sim.simxFinish(-1)  # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to V-REP
if clientID != -1:
    print('Connected to remote API server')

    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    # res,objs=sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking)
    # if res==sim.simx_return_ok:
    #    print ('Number of objects in the scene: ',len(objs))
    # else:
    #    print ('Remote API function call returned with error code: ',res)
    time = 0
    # retrieve motor  handles
    errorCode, left_motor_handle = sim.simxGetObjectHandle(clientID, 'wheel_left_joint', sim.simx_opmode_blocking)
    errorCode, right_motor_handle = sim.simxGetObjectHandle(clientID, 'wheel_right_joint', sim.simx_opmode_blocking)
    r, signalValue = sim.simxGetFloatSignal(clientID, 'Turtlebot2_simulation_time', sim.simx_opmode_streaming)
    video = [[5, 5], [10, 10], [10, 5], [5, 10], [10, 5], [10, 10], [10, 5], [5, 10], [10, 0], [5, 10], [10, 5], [0, 10], [5, 10], [10, 5], [5, 10], [10, 5], [5, 10], [5, 0], [0, 5], [5, 0], [0, 10], [10, 5], [0, 10], [10, 0], [5, 0], [0, 5], [10, 5], [5, 10], [10, 10], [10, 10], [10, 10], [10, 10], [10, 10], [10, 5], [0, 10]]

    rev1 = video[::-1]
    for k in rev1:
        time = 0
        err_code1 = 1
        err_code2 = 2
        # print(type(k[0]))
        while (err_code1 != 0 and err_code2 != 0):
            err_code1 = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, k[0], sim.simx_opmode_streaming)
            # print(err_code1)

            err_code2 = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, k[1], sim.simx_opmode_streaming)
            # print(err_code2)

        r, signalValue = sim.simxGetFloatSignal(clientID, 'Turtlebot2_simulation_time', sim.simx_opmode_buffer)

        while (time <= 1):
            r, signalValue2 = sim.simxGetFloatSignal(clientID, 'Turtlebot2_simulation_time', sim.simx_opmode_buffer)

            time = signalValue2 - signalValue


    errorCode = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, 0, sim.simx_opmode_streaming)
    errorCode = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, 0, sim.simx_opmode_streaming)

    
    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to V-REP:
    sim.simxFinish(clientID)
else:
    print('Failed connecting to remote API server')
print('Program ended')