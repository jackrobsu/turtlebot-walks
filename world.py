#-*- coding:utf-8 -*-
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

from test import Map,Cell

class World(object):
    def __init__(self):
        self.map = [
            ['-','-','g','-'],
            ['.','.','.','.'],
            ['.','b','-','.'],
            ['.','.','.','.'],
        ]
        self.Map = None
        self.path = []
        self.Target = None
        self.rows = len(self.map)
        self.cols = len(self.map[0])
        self.traps = [0,1,3,10]
        self.walls = [9]
        self.targets = [2]
        self.legalstate = filter(lambda x:not ( x in self.traps or x in self.targets or x in self.walls )  , [ i for i in range(self.rows * self.cols)])
        # print(self.legalstate)

        self.actionmap = {
            0:'up',
            1:'down',
            2:'left',
            3:'right'
        }
        self.targetReward = 1.0
        self.trapReward = -1.0
        self.wallReward = -0.5
        self.norReward = -0.1
        self.map_to_state , self.state_to_map = self.init()
        self.stateNumber = len(self.map_to_state)
        self.legalstate = map(lambda x:self.map_to_state[x],self.legalstate)
        # print(self.legalstate)

    def init(self):
        map_to_state = {}
        state_to_map = {}
        
        mapindex = 0
        stateindex = 0
        for i in range(self.rows) :
            for j in range(self.cols) :
                if self.map[i][j] != 'b' :
                    map_to_state[mapindex] = stateindex
                    state_to_map[stateindex] = mapindex
                    stateindex += 1
                mapindex += 1
        return map_to_state , state_to_map
        print(len(state_to_map))
        print(map_to_state)
        print(state_to_map)
        

    def simulation(self,state,action):
        if state not in self.state_to_map :
            print("exception: illegal state")
            exit(0)
        m_state = self.state_to_map[state]
        nextstate = self.__execute(m_state,action)
        c = 0
        r = 0.0
        if self.isLegalState(nextstate):
            c , r = self.getReward(nextstate)
        else:
            r = self.norReward
            c = 0
            nextstate = m_state
        if nextstate in self.walls :
            nextstate = m_state
        return [state,action,self.map_to_state[nextstate],r,c]
    def __execute(self,state,action):
        if action == 0 :
            nextstate = state - self.rows
        elif action == 1 :
            nextstate = state + self.rows
        elif action == 2 :
            nextstate = state - 1
        elif action == 3 :
            nextstate = state + 1
        else:
            nextstate = state
        return nextstate

    
    def isTarget(self,state):
        if state < 0 or state >= self.rows * self.cols:
            return False
        state = self.state_to_map[state]
        row = state / self.cols
        col = state - row * self.cols
        if self.map[row][col] == 'g':
            return True
        return False
            

    def getReward(self,state):
        if state in self.targets :
            return (-1,self.targetReward)
        elif state in self.traps :
            return (-1,self.trapReward)
        elif state in self.walls :
            return (0,self.wallReward)
        else:
            return (0,self.norReward)

    def isLegalState(self,state):
        if state >= 0 and state <= self.rows * self.cols - 1 :
            return True
        else:
            return False
        
    def state_to_coordinate(self,state):
        if self.isLegalState(state) :
            row = state / self.cols
            col = state - self.cols * row
            return (row+1,col+1)
        else:
            return (None,None)

    def setMap(self,map):
        self.Map = map
        self.Map.gridsize = self.Map.gridsize * self.Map.grids
        self.rows = int(self.Map.height / self.Map.grids) + 1
        self.cols = int(self.Map.width / self.Map.grids) + 1
        self.stateNumber = self.cols * self.rows
        print(self.Map.height,self.Map.width)
        print(self.rows,self.cols)
        print(self.stateNumber)
        print(self.Map.turtlebotPosition)
        print(self.getStateNum(self.Map.getCell(self.Map.turtlebotPosition)))
    
    def getPosition(self):
        if self.Map == None :
            return None
        else:
            return self.getStateNum(self.Map.getCell(self.Map.turtlebotPosition))

    def doAction(self,action):
        if action == 0 :
            x = 0
            y = 1
        elif action == 1 :
            x = 0
            y = -1
        elif action == 2 :
            x = -1
            y = 0
        elif action == 3 :
            x = 1
            y = 0
        else:
            x = y = 0

        return x , y

    #把机器人发来的动作转换成地图中的下一个目标位置，然后发送给move_base
    #同时接收返回的数据，包括reward、当前位置等，然后通过一定的reward机制评定最后的reward，最后返回给机器人
    def doSimulation(self,action):
        x , y = self.doAction(action)
        state = self.Map.sendGoal(x,y)
        if state != None :
            state.Print()
        # time.sleep(1)
        state1 = self.getStateNum(state.curCell)
        state2 = self.getStateNum(state.NextCell)
        realstate = self.getStateNum(state.realCell)

        targetCell = self.getCellFromStateNum(self.Target)
        
        diff1 = math.fabs(targetCell.X-state.curCell.X) + math.fabs(targetCell.Y-state.curCell.Y) 
        diff2 = math.fabs(targetCell.X-state.NextCell.X) + math.fabs(targetCell.Y-state.NextCell.Y)

        addreward = 0
        extraReward = 0.0
        if diff2 >= diff1 :
            extraReward += 50

        #原地不懂，惩罚，表明可能遇到了障碍物
        if state1 == state2 :
            extraReward += 10
        
        #离目标越近，reward越高
        if math.fabs( targetCell.X - state.curCell.X ) > math.fabs(targetCell.X - state.NextCell.X) :
            addreward += 20

            if math.fabs(targetCell.X - state.NextCell.X) <= 10 :
                addreward += 5
            elif math.fabs(targetCell.X - state.NextCell.X) <= 8 :
                addreward += 5
            elif math.fabs(targetCell.X - state.NextCell.X) <= 6 :
                addreward += 7
            elif math.fabs(targetCell.X - state.NextCell.X) <= 4 :
                addreward += 9
            elif math.fabs(targetCell.X - state.NextCell.X) <= 2 :
                addreward += 11
        
        if math.fabs( targetCell.X - state.curCell.X ) < math.fabs(targetCell.X - state.NextCell.X) :
            addreward -= 20

        if math.fabs( targetCell.Y - state.curCell.Y ) > math.fabs(targetCell.Y - state.NextCell.Y) :
            addreward += 20

            if math.fabs(targetCell.Y - state.NextCell.Y) <= 10 :
                addreward += 5
            elif math.fabs(targetCell.Y - state.NextCell.Y) <= 8 :
                addreward += 5
            elif math.fabs(targetCell.Y - state.NextCell.Y) <= 6 :
                addreward += 7
            elif math.fabs(targetCell.Y - state.NextCell.Y) <= 4 :
                addreward += 9
            elif math.fabs(targetCell.Y - state.NextCell.Y) <= 2 :
                addreward += 11
            
        if math.fabs( targetCell.Y - state.curCell.Y ) < math.fabs(targetCell.Y - state.NextCell.Y) :
            addreward -20
            
        
            
        state.reward -= extraReward
        state.reward += addreward        
        print("a reward ",state.reward)
        self.path.append([state1,action,state2,state.reward,state.c])
        flag = self.checkGoal(state2)
        if flag :
            for i in range(self.path):
                self.path[i][3] += 10

        if flag :
            return realstate , False , self.path , state.reward , action
        else:
            return realstate , True , self.path , state.reward , action
        
    def checkGoal(self,state):
        if state == self.Target :
            print("get target ",self.getCellFromStateNum(self.Target))
            return True
        return False

    def clearPath(self):
        self.path = []
    
    #根据地图中的cell获取状态编号
    def getStateNum(self,cell):
        num = cell.Y * self.cols + cell.X
        return num

    #根据状态编号获取地图中cell
    def getCellFromStateNum(self,num):
        y = num / self.cols
        x = num - self.cols * y
        cell = Cell(x,y)
        return cell

    def getPositionStateNum(self,pos):
        if self.Map != None :
            print(self.getStateNum(self.Map.getCell(pos)))



# mapspace = Map()
# world = World()
# def getMap(data):
#     global mapspace
#     data = data.data
#     mapspace.setMap(data)
    
    
# def getMapMetaData(data):
#     global mapspace
#     mapspace.setMapParameters(data)
#     # mapspace.Print()

# def getPosition(data):
#     # print(data)
#     global mapspace , world
#     mapspace.setPosition(data.pose.pose.position)
#     world.getPositionStateNum(data.pose.pose.position)



# if __name__ == '__main__' :
#     # world = World()
#     # print(world.cols)
#     # state = 2
#     # print(world.state_to_coordinate(state))
#     # print(world.simulation(state,1))
#     # print(world.state_to_coordinate(state))

#     # print(world.state_to_coordinate(3))
        
#     posePub = rospy.Publisher("/move_base_simple/goal",PoseStamped,queue_size=10)
#     modelpositionPub = rospy.Publisher("/gazebo/set_model_state",ModelState,queue_size=10)

#     rospy.init_node("test",anonymous=False)
#     rospy.Subscriber("/map",OccupancyGrid,callback=getMap)
#     rospy.Subscriber("/map_metadata",MapMetaData,getMapMetaData)
#     rospy.Subscriber("/amcl_pose",PoseWithCovarianceStamped,getPosition)

#     mapspace.init()
#     # sim = rospy.get_param('/use_sim_time')
#     # print(sim)
    
#     while not mapspace.ready() :
#         pass
#     print("ready")
#     mapspace.getNewMap()
#     world.setMap(mapspace)

#     # times = 0
#     # while True:
#     #     state = mapspace.sendGoal(1,0)
#     #     if state != None :
#     #         state.Print()
#     #     print("send goal %d" % times)
#     #     times += 1
#     #     time.sleep(1)
#     #     if times > 10 :
#     #         break
#     #     pass
 
#     rospy.spin()