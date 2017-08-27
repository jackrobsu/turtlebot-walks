#-*- coding:utf-8 -*-
import roslib
import rospy
import math
import time
import actionlib  
import copy

from actionlib_msgs.msg import *  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Pose , Point , PoseWithCovarianceStamped , PoseStamped , Twist
from gazebo_msgs.msg import ModelState

class Cell:
    def __init__(self,x=None,y=None):
        self.X = x
        self.Y = y

    def doVariationOnX(self,var):
        self.X += var

    def doVariationOnY(self,var):
        self.Y += var


class State:
    def __init__(self):
        self.curCell = None
        self.NextCell = None
        self.realCell = None
        self.reward = -1
        self.c = True

    def setCurrentCell(self,cell):
        self.curCell = copy.deepcopy(cell)

    def setNextCell(self,cell):
        self.NextCell = copy.deepcopy(cell)

    def Print(self):
        # print("pos ",self.pos,self.nextPos)
        print("curcell")
        if self.curCell != None :
            print(self.curCell.X,self.curCell.Y)
        else:
            print("None")
        
        print("nextcell")
        if self.NextCell != None :
            print(self.NextCell.X,self.NextCell.Y)
        else:
            print("None")

        print("reward ",self.reward)

    def clear(self):
        self.curCell = None
        self.NextCell = None
        self.realCell = None
        self.reward = -1
        self.c = True
        

class Map(object):

    def __init__(self):
        self.gridsize = None
        self.grids = 3
        self.mapdata = None
        self.width = None
        self.height = None
        self.mapPosition = None
        self.mapPoint = None
        self.turtlebotPosition = None
        self.oldState = State()
 

 

        self.poseSeq = 0

    def init(self):
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  
        self.move_base.wait_for_server()
        self.goal = MoveBaseGoal()  
        self.goal.target_pose.header.frame_id = 'map'  

        
    def setMap(self,data):
        self.mapdata = data

    def setMapParameters(self,data):
        self.mapPosition = data.origin
        self.mapPoint = data.origin.position
        self.width = data.width
        self.height = data.height
        self.gridsize = data.resolution
        
    def getNewMap(self):
        if self.mapdata == None :
            return
        
    def setPosition(self,position):
        self.turtlebotPosition = position
        # self.checkPosition(position)
        # print(position)

    def In(self,state,position):
        if self.gridsize == None :
            return True
        inX = False
        inY = False
        if position.x >= state.pos.x and position.x <= state.pos.x + self.gridsize * self.grids :
            inX = True

        if position.y >= state.pos.y and position.y <= state.pos.y + self.gridsize * self.grids :
            inY = True
        
        if inX and inY :
            return True
        
        return False
    
    def checkPosition(self,position):
        if not self.In(self.oldState,position):
            self.turtlebotPosition = position
            self.oldState.setCurrentCell(self.getCell(position))
            self.oldState.Print()
        


    def getCell(self,position=None):
        if position == None :
            position = self.turtlebotPosition

        x , y , z = self.getXYZ(position)
        mapX , mapY , mapZ = self.getXYZ(self.mapPoint)

        x = int(math.fabs(x-mapX) / (self.gridsize*self.grids))
        y = int(math.fabs(y-mapY) / (self.gridsize*self.grids))
        cell = Cell(x,y)
        print(cell.X,cell.Y)
        
        return cell
        
    def getXYZ(self,position):
        return position.x , position.y , position.z

    def getCellPos(self,cell):
        mapX , mapY , mapZ = self.getXYZ(self.mapPoint)
        x = mapX + self.gridsize * self.grids * cell.X
        y = mapY + self.gridsize * self.grids * cell.Y
        return x , y
        

    def getPose(self,cell):
        mapX , mapY , mapZ = self.getXYZ(self.mapPoint)
        x = ( cell.X + 0.5 ) * self.gridsize * self.grids + mapX
        y = ( cell.Y + 0.5 ) * self.gridsize * self.grids + mapY
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        return pose

    def sendGoal(self,xd,yd):
        cell = self.getCell()
        self.oldState.setCurrentCell(cell)
        print("cur pose ",self.turtlebotPosition.x,self.turtlebotPosition.y)
        print("curcell ",cell.X,cell.Y)
        cell.doVariationOnX(xd)
        cell.doVariationOnY(yd)
        print("nextcell ",cell.X,cell.Y)
        self.oldState.setNextCell(cell)
        pose = self.getPose(cell)
        pose.orientation.w = 1
        print("next pose ",pose.position.x,pose.position.y)
        self.goal.target_pose.header.stamp = rospy.Time.now()  
        self.goal.target_pose.pose = pose
        self.move_base.send_goal(self.goal)  
        
        return self.waitForServer()
    

    def waitForServer(self):
        self.move_base.wait_for_result(rospy.Duration(8))
        state = self.move_base.get_state()  
        # print(state,GoalStatus.SUCCEEDED)
        cState = copy.deepcopy(self.oldState) 
        cell = self.getCell()
        cState.realCell = cell
        if state == GoalStatus.SUCCEEDED:  
            # self.checkPosition(self.turtlebotPosition)
            rospy.loginfo("Goal succeeded!")  
        else:
            self.move_base.cancel_goal()
           
            cState.reward = -10

            rospy.loginfo("failed")

        self.oldState.clear()
        return cState

    def Print(self):
        print(self.mapPoint)
        print(self.width)
        print(self.height)
        print(self.gridsize)
        print(self.grids)

    def ready(self):
        if self.mapdata == None or self.mapPosition == None or self.turtlebotPosition == None :
            return False
        return True



# if __name__ == '__main__' :
    
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
#     times = 0
#     while( mapspace.sendGoal() ):
#         print("send goal %d" % times)
#         times += 1
#         time.sleep(1)
#         if times > 1000 :
#             break
#         pass

#     # model_state_msg = ModelState()
#     # model_state_msg.model_name = "mobile_base"
#     # model_state_msg.pose.position.x = 2
#     # model_state_msg.pose.position.y = 0
#     # model_state_msg.pose.position.z = 2
#     # # quaternion = quaternion_from_euler(0, 0, angle)
#     # # model_state_msg.pose.orientation.x = quaternion[0]
#     # # model_state_msg.pose.orientation.y = quaternion[1]
#     # # model_state_msg.pose.orientation.z = quaternion[2]
#     # # model_state_msg.pose.orientation.w = quaternion[3]
#     # model_state_msg.reference_frame = "world"
#     # modelpositionPub.publish(model_state_msg)
#     # # ms = ModelState()
#     # # ms.model_name = "mobile_base"
#     # # ms.pose.position.x = 1
#     # # ms.pose.position.y = 1
#     # # ms.pose.orientation.w = 1
#     # # ms.reference_frame = "world"
#     # # modelpositionPub.publish(ms)
#     # # print(ms)
   
 
#     rospy.spin()