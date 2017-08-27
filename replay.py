#-×- coding:utf-8 -*-
import roslib
import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Empty as EmptyMsg
from std_msgs.msg import String


timePub = None
startPub = None
g_set_state = None
pub = None
def main():
    global startPub , timePub , g_set_state , pub

    rospy.init_node('move_robot_to_given_place')
    g_set_state = rospy.ServiceProxy("/gazebo/set_model_state",SetModelState)
    pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped,latch=True,queue_size=10)
    startPub = rospy.Publisher("startTime",String,queue_size=10)
    timePub = rospy.Publisher("TimeOut",String,queue_size=10)
    rospy.Subscriber("startTime",String,timesleep)
    rospy.Subscriber("StartAgain",String,startAgain)
    rospy.Subscriber("Stop",String,Stop)
    
    rospy.spin()
      
#定时作用，即一定时间后发送超时消息
def timesleep(data):
    global timePub
    rospy.sleep(80)
    timePub.publish(String("0"))

#重新启动定时
def startAgain(data):
    global startPub
    startPub.publish(String("0"))

#使机器人回到原位
def Stop(data):
    global g_set_state , pub

    pose = Pose()
    
    pose.position.x = 0
    pose.position.y = 0
    pose.position.z = 0
    
    pose.orientation.w = 1.0
    
    state = ModelState()
    
    state.model_name = "mobile_base"
    state.pose = pose
    
    
    rospy.loginfo("Moving robot")
    try:
            
      ret = g_set_state(state)
      
      print ret.status_message
            
    except Exception, e:
        
      rospy.logerr('Error on calling service: %s',str(e))
    
    
    loc = PoseWithCovarianceStamped()
    
    loc.pose.pose = pose
    loc.header.frame_id = "/map"
    #loc.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
    
    rospy.loginfo("Adjusting localization")
    pub.publish(loc)
    pub.publish(loc)
    pub.publish(loc)
    
    rospy.sleep(1)
    

if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException: pass
