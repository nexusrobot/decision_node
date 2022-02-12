#!/usr/bin/env python
# -*- coding: utf-8 -*-

## Multiarrayのpub
## https://asukiaaa.blogspot.com/2021/07/rostopic-pub-float32multiarray.html
## rostopic pub -r 1 Info_enemy std_msgs/Float32MultiArray "data: [20.0, 20.3]"

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int8
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
from enum import IntEnum

#BasicRun
from geometry_msgs.msg import Twist
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs
#from decision_node.scripts import Condition
import math as m

#from algorithm import BasicRun
#from algorithm import BasicRun

# /Info_enemy
# Float32MultiArray
# 1st elem : enemy distance 0-100  notfound=-1
# 2nd elem : enemy direction right=+100 left=-100 center=0

# /Info_obstacle
# Float32MultiArray
# 1st elem : distance
# 2nd elem : direction


class InfoEnemyIdx(IntEnum):
    INDEX_DIS = 0
    INDEX_DIR = 1

class HelloCvIdx(IntEnum):
    INDEX_CX = 0
    INDEX_CY = 1
    INDEX_AREA = 2

class EnemyAttitudeIdx(IntEnum):
    ERROR = -1
    INVISIBLE = 0
    FRONT = 1
    SIDE = 2


#敵がこちらを向いているか尻を向けているか
class EnemyCondition:
    def __init__(self):
        self.dist = -9999
        self.green_area = -1
        self.info_enemy_sub = rospy.Subscriber('Info_enemy', Float32MultiArray, self.callback_enemy)
        self.green_sub = rospy.Subscriber('green_center', Int32MultiArray, self.callback_greenCenter)
        self.dist_area = [
            ( 10,3540000),
            ( 20,3180000),
            ( 30,2660000),
            ( 40,2426000),
            ( 50,1300000),
            ( 60,950000),
            ( 70,570000),
            ( 80,443000),
            ( 90,262500),
            (100,144500),
        ]
        self.tuningRatio = 1.0 #Ratio<1.0でFRONTと判定される範囲が狭くなる。

    def getAttitude(self):
        if self.dist == -9999:
            return EnemyAttitudeIdx.ERROR
        if self.green_area == -1:
            return EnemyAttitudeIdx.ERROR

        if self.dist < 0:
            return EnemyAttitudeIdx.INVISIBLE

        for d,a in self.dist_area:
            if d > self.dist:
                hokan_a = (a * d / self.dist)
                area_th = hokan_a * self.tuningRatio
                rospy.loginfo("dist={} area={} area_th={}".format(d, a, area_th))
                break
        
        if area_th < self.green_area:
            return EnemyAttitudeIdx.SIDE
        else:
            return EnemyAttitudeIdx.FRONT
        

    def callback_enemy(self,Info_enemy):
        self.dist = Info_enemy.data[InfoEnemyIdx.INDEX_DIS]

    def callback_greenCenter(self,green_center):
        self.green_area = green_center.data[HelloCvIdx.INDEX_AREA]



#周囲の状況
class Condition:
    
    def __init__(self):
        self.info_enemy_sub = rospy.Subscriber('Info_enemy', Float32MultiArray, self.callback_enemy)
        self.info_obstacle_sub = rospy.Subscriber('Info_obstacle', Float32MultiArray, self.callback_obstacle)
        self.enemyCondition = EnemyCondition()

    def update(self):
        #rospy.loginfo("attitude %s",self.enemyCondition.getAttitude())
        return

    def getEnemyAttitude(self):
        return self.enemyCondition.getAttitude()
         
        
    def callback_enemy(self,Info_enemy):
        #rospy.loginfo("ememy dis=%f",Info_enemy.data[InfoEnemyIdx.INDEX_DIS])
        #rospy.loginfo("ememy dir=%f",Info_enemy.data[InfoEnemyIdx.INDEX_DIR])
        self.Info_enemy = Info_enemy
        self.update()

    def callback_obstacle(self,Info_obstacle):
        #rospy.loginfo("obstacle dis=%f",Info_obstacle.data[InfoEnemyIdx.INDEX_DIS])
        #rospy.loginfo("obstacle dir=%f",Info_obstacle.data[InfoEnemyIdx.INDEX_DIR])
        self.Info_obstacle = Info_obstacle
        self.update()



class Decision:
    def __init__(self):
        rospy.loginfo("Decision node")
        self.condition = Condition()
        self.algo = BasicRun()

    def run(self):
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            self.algo.execute(self.condition)
            rate.sleep()



class WayPoint():
    def __init__(self):
        self.goal_table = [
            ( 0.53,    0,      0),
            (-0.53,    0,   m.pi),
        ]
        self.idx = 0

    def loadPoint(self):
        while True:
            p = self.goal_table[self.idx]
            self.idx += 1
            if self.idx > len(self.goal_table)-1:
                self.idx = 0
            yield p

    def getWayPoint(self):
        p = self.loadPoint()
        return p.next()


class BasicRun():
    def __init__(self):
        self.wayPoint = WayPoint()
        
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("BasicRun")

    def execute(self, condition):
        r = rospy.Rate(5) # change speed 5fps

        x,y,th = self.wayPoint.getWayPoint()
        rospy.loginfo("waypoint : {} {} {}".format(x,y,th))
        result = self.setGoal(x,y,th)
        rospy.loginfo("waypoint result : {}".format(result))

        if condition.getEnemyAttitude() == EnemyAttitudeIdx.FRONT:
            return False
        else:
            return True




    def setGoal(self,x,y,yaw):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,yaw)        
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()        



if __name__ == '__main__':
    rospy.init_node('DecisionNode')
    node = Decision()
    node.run()




