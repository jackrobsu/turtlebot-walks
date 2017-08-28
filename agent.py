#-*- coding:utf-8 -*-

import numpy as np
import random as rand
import time
from MDPy import *
from world import *

import roslib
import rospy
import math
import time
import actionlib  
from actionlib_msgs.msg import *  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Pose , Point , PoseWithCovarianceStamped , PoseStamped , Twist
from gazebo_msgs.msg import ModelState
from std_msgs.msg import String

from test import Map

class Agent(object):
    def __init__(self,Map):
        self.samples = []
        self.transitions = []
        self.unknownlabel = []
        self.world = World()
        self.world.setMap(Map)
        self.states = self.world.stateNumber
        print("states ",self.states)
        self.actions = len(self.world.actionmap)
        self.policy = None
        self.rmax = 10.0
        self.state0 = self.states
        self.init()
        self.followProb = 1.0
        self.stop = False

    def initSample(self):
        self.samples = []
        for i in range(self.states):
            self.samples.append([])
            self.samples[i] = []
            for j in range(self.actions) :
                self.samples[i].append([])

    def init(self):
        self.initSample()
        # print(self.samples)
        for i in range(self.states):
            self.transitions.append([])
            self.unknownlabel.append([])
            self.transitions[i] = []
            self.unknownlabel[i] = []
            for j in range(self.actions) :
                self.transitions[i].append([])
                self.transitions[i][j].append([self.state0,self.rmax,1.0,0.0])
                self.unknownlabel[i].append(False)

        self.policy = self.getPolicy()
        # print(self.policy)
        # print(self.transitions)

    def getPolicy(self):
        mdp = MDP()
        mdp.add_states(self.states+1)
        for i in range(self.states):
            mdp.add_actions(i,self.actions)
            for j in range(self.actions):
                for transition in self.transitions[i][j] :
                    mdp.add_transition(i,j,transition[0:3])
        mdp.value_iteration(0.5)
        return mdp.policy

    # 通过samples更新策略
    def update(self):
        for i in range(self.states):
            sum = 0.0
            for j in range(self.actions):
                sum += len(self.samples[i][j])
            if sum == 0.0 :
                continue
            for j in range(self.actions):
                if len(self.samples[i][j]) == 0 :
                    continue
                if not self.unknownlabel[i][j] :
                    self.unknownlabel[i][j] = True
                    self.transitions[i][j] = []
                states = {}                          
                for item in self.samples[i][j] :
                    if item[0] not in states :
                        states[item[0]] = [item[1],1]
                    else:
                        states[item[0]][0] += item[1]
                        states[item[0]][1] += 1

                for s in states :
                    flag = False
                    for index in range(len(self.transitions[i][j])) :
                        if s == self.transitions[i][j][index][0] :
                            self.transitions[i][j][index][1] = float( self.transitions[i][j][index][1] * self.transitions[i][j][index][3] + states[s][0] ) / float( self.transitions[i][j][index][3] + states[s][1] )
                            self.transitions[i][j][index][3] += states[s][1] 
                            self.transitions[i][j][index][2] = float(states[s][1]) / float(sum)
                            flag = True
                            break
                    if flag == False :
                        self.transitions[i][j].append([s,states[s][0]/states[s][1],states[s][1]/sum,states[s][1]])
                # print("states ",states)
                # print(sum)
                # print(i,j)
                # print(self.transitions[i][j])
                # exit(0)
        print(self.state0,self.states)
        for i in range(self.states):
            for j in range(self.actions):
                for transition in self.transitions[i][j] :
                    if transition[0] != self.state0 :
                        print(transition)
        # print(self.transitions)
        self.policy = self.getPolicy()
        if self.followProb > 0.3 :
            self.followProb = self.followProb * 0.99

    def executeAction(self,state):
        r = self.world.simulation(state,self.policy[state])
        self.samples[r[0]][r[1]].append(r[2:4])
        return r[2:5]

    # 向世界发送动作消息
    def doAction(self,state,action=None,reward=None):
        if action == None :
            return self.world.doSimulation(self.policy[state])
        else:
            p = rand.random()
            #如果上一步动作执行完获得了不错的reward，那么一定概率再次选择与上次一致的动作进行探索（目前只适用于导航）
            if p < self.followProb :
                isSame = False
                
                if reward != None :
                    if reward > 0 :
                        prob = rand.random()
                        if reward > 5 :
                            if prob <= 0.5 :
                                isSame = True
                        elif reward >= 9 :
                            if prob <= 0.75 :
                                isSame = True
                        elif reward >= 15 :
                            if prob <= 0.85 :
                                isSame = True
                        else:
                            isSame = True

                if isSame :
                    print("follow the last action %s" % self.world.actionmap[action])
                    return self.world.doSimulation(action)
                else:
                    return self.world.doSimulation(self.policy[state])
            else:
                return self.world.doSimulation(self.policy[state])
                
                    
                    

    def getTarget(self,state):
        return self.world.isTarget(state)

    def setTarget(self):
        self.world.Target = 404

    def getInitPosition(self):
        # return self.world.legalstate[np.random.randint(0,len(agent.world.legalstate))]
        p = self.world.getPosition()
        while p == None :
            p = self.world.getPosition()
        return p

    def Timeout(self):
        self.stop = True

    def printPolicy(self):
        for p in self.policy:
            cell = self.world.getCellFromStateNum(p)
            if cell.X >=4 and cell.X <= 16 and cell.Y >= 4 and cell.Y <= 16 :
                print( " ( %d , %d ) , %d %s " % ( cell.X,cell.Y,self.policy[p],self.world.actionmap[self.policy[p]] ) ) 

    def checkSamples(self):
        if len(self.samples) > 10000 :
            del self.samples[0:100]

agent = None
mapspace = Map()

#接收地图数据
def getMap(data):
    global mapspace
    data = data.data
    mapspace.setMap(data)
    
#接收地图规格数据
def getMapMetaData(data):
    global mapspace
    mapspace.setMapParameters(data)
    # mapspace.Print()

#实时获得机器人当前位置
def getPosition(data):
    # print(data)
    global mapspace
    mapspace.setPosition(data.pose.pose.position)

#超时消息处理
def TimeOut(data):
    global agent
    agent.Timeout()

Start = None
Stop = None

def startNode():
    global Start , Stop
    Start = rospy.Publisher("StartAgain",String,queue_size=10)
    Stop = rospy.Publisher("Stop",String,queue_size=10)

    rospy.init_node("test",anonymous=False)
    rospy.Subscriber("/map",OccupancyGrid,callback=getMap)
    rospy.Subscriber("/map_metadata",MapMetaData,getMapMetaData)
    rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped,getPosition)
    rospy.Subscriber("TimeOut",String,TimeOut)

if __name__ == '__main__' :
    startNode()

    mapspace.init()
    while not mapspace.ready() :
        pass
    print("ready")
    mapspace.getNewMap()
    # world.setMap(mapspace)

    agent = Agent(mapspace)
    agent.setTarget()
    # agent.getPolicy()
    state = agent.getInitPosition()     
    print("state ",state)
    # print(agent.policy)
    agent.printPolicy()
    Start.publish(String("Start"))
    successtimes = 0
    times = 0

    #循环执行机器人的动作
    for _ in range(1000):
        action = None
        reward = 0.0
        cost = 0
        count = 0
        for i in range(10000):
            
            nextstate , c , path , cost , action = agent.doAction(state,action,cost)
            if c == True :
                state = nextstate
                print("state num",state)
                print("cell ",agent.world.getCellFromStateNum(state).X,agent.world.getCellFromStateNum(state).Y)

                if agent.stop :
                    print("timeout")
                    times += 1
                    for r in path :
                        # r[3] -= 5
                        print("sample ",r)
                        agent.samples[r[0]][r[1]].append(r[2:4])
                    cell = agent.world.getCellFromStateNum(agent.world.Target)
                    print("target ",cell.X,cell.Y)
                    agent.stop = False
                    break

            else:
                successtimes += 1
                times += 1
                for r in path :
                    print("sample ",r)
                    
                    agent.samples[r[0]][r[1]].append(r[2:4])
                break
                state = agent.getInitPosition()

            # nextstate ,r ,c = agent.executeAction(state)
            # reward += r
            # if c == -1 :
            #     if agent.getTarget(nextstate) :
            #         print('reward ',reward)
            #         print('step ',count)
            #         successtimes += 1
            #     state = agent.getInitPosition()
            #     count = 0agent.getTarget(nextstate) :
            #         print('reward ',reward)
            #         print('step ',count)
            #         successtimes += 1
            #     state = agent.getInitPosition()
            #     count = 0
            #     times += 1
            #     reward = 0.0
            # else:
            #     state = nextstate
            #     count += 1
        agent.world.clearPath()
        agent.update()
        print("update")
        agent.printPolicy()
        agent.initSample()
        Stop.publish(String("Stop"))
        time.sleep(5)
        agent.getInitPosition()
        Start.publish(String("Start"))
    print('policy ',agent.policy)
    print('success rate %d / %d = %.10f' % (successtimes, times,float(successtimes)/float(times)))
    # print(agent.samples)

