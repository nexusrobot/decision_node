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

#from algorithm import BasicRun
from algorithm import BasicRun

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
        return self.getEnemyAttitude()
         
        
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





if __name__ == '__main__':
    rospy.init_node('DecisionNode')
    node = Decision()
    node.run()




