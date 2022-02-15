#!/usr/bin/env python
# -*- coding: utf-8 -*-
# nexusrobot

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
import math as m

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

        area_th = 0
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
        self.attitude_pub = rospy.Publisher('attitude_enemy', String, queue_size=1)

    def update(self):
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


class TurnDirection(IntEnum):
    RIGHT_TURN = 1
    LEFT_TURN = -1

class WayPoint():
    def __init__(self,waypoint_file):
        self.idx = 0
#        self.RIGHT_TURN = 1 #const
#        self.LEFT_TURN = -1 #const

        with open(waypoint_file,"r") as fp:
            lines = fp.read().splitlines()
            self.waypoints = [tuple(map(float,line.split(','))) for line in lines]
        

    def getNext(self,turn_direction):
        max_idx = len(self.waypoints)-1

        self.idx += turn_direction
        if self.idx > max_idx:
            self.idx = 0
        elif self.idx < 0:
            self.idx = max_idx
        return self.getCurrent()     

    def getCurrent(self):
        return self.waypoints[self.idx]


class BasicRun():
    def __init__(self):
        self.wayPoint = WayPoint()
        
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        rospy.loginfo("BasicRun")

        x,y,th = self.wayPoint.getWayPoint()
        rospy.loginfo("waypoint : {} {} {}".format(x,y,th))
        result = self.setGoal(x,y,th)
    

    def runActionlib(self):
        self.status = self.move_base_client.get_state()
        rospy.loginfo("actionlib status {}".format(self.status))

        if self.status == actionlib.GoalStatus.ACTIVE:
            rospy.loginfo("active")

        elif self.status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("load next goal")
            x,y,th = self.wayPoint.getWayPoint()
            rospy.loginfo("waypoint : {} {} {}".format(x,y,th))
            result = self.setGoal(x,y,th)
        
        elif self.status == actionlib.GoalStatus.PENDING:
            x,y,th = self.wayPoint.getCurrentWayPoint()
            self.setGoal(x,y,th)

        elif self.status == actionlib.GoalStatus.ABORTED:
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.5
            self.direct_twist_pub.publish(cmd_vel)

    ## actionlib.GoalStatus.???
    ## https://docs.ros.org/en/noetic/api/actionlib_msgs/html/msg/GoalStatus.html
    def execute(self, condition):
        r = rospy.Rate(5) # change speed 5fps

        attitude = condition.getEnemyAttitude()
        if attitude == EnemyAttitudeIdx.FRONT or attitude == EnemyAttitudeIdx.SIDE:
            rospy.loginfo("Enemy found. attitude = {}".format(attitude))
            self.cancel_goal()
            return False
        else:
            self.runActionlib()
            return True

    def setGoal(self,x,y,yaw):
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

        self.move_base_client.send_goal(goal)
        rospy.sleep(0.5)
        rospy.loginfo("sendGoal:{},{},{}".format(x,y,yaw))
    
    def cancel_goal(self):
        self.move_base_client.cancel_all_goal()
        return



if __name__ == '__main__':
#    rospy.init_node('DecisionNode')
#    node = Decision()
#    node.run()

    rospy.init_node('test')
    way = WayPoint('course1.txt')
    rospy.loginfo(way.getCurrent())

    for i in range(8):
        rospy.loginfo(way.getNext(TurnDirection.RIGHT_TURN))
    for i in range(8):
        rospy.loginfo(way.getNext(TurnDirection.LEFT_TURN))




