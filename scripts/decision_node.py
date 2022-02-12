#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int8
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from enum import IntEnum

# /Info_enemy
# Float32MultiArray
# 1st elem : enemy distance 0-100  notfound=-1
# 2nd elem : enemy direction right=+100 left=-100 center=0

# /Info_obstacle
# Float32MultiArray
# 1st elem : distance
# 2nd elem : direction


class IndexSearch(IntEnum):
    INDEX_DIS = 0
    INDEX_DIR = 1

#周囲の状況
class Condition:
    
    def __init__(self):
        self.info_enemy_sub = rospy.Subscriber('Info_enemy', Float32MultiArray, self.callback_enemy)
        self.info_obstacle_sub = rospy.Subscriber('Info_obstacle', Float32MultiArray, self.callback_obstacle)

    def callback_enemy(self,Info_enemy):
        rospy.loginfo("ememy dis=%f",Info_enemy.data[IndexSearch.INDEX_DIS])
        rospy.loginfo("ememy dir=%f",Info_enemy.data[IndexSearch.INDEX_DIR])
        self.Info_enemy = Info_enemy

    def callback_obstacle(self,Info_obstacle):
        rospy.loginfo("obstacle dis=%f",Info_obstacle.data[IndexSearch.INDEX_DIS])
        rospy.loginfo("obstacle dir=%f",Info_obstacle.data[IndexSearch.INDEX_DIR])
        self.Info_obstacle = Info_obstacle


class Decision:
    def __init__(self):
        rospy.loginfo("Decision node")
        self.condition = Condition()

    def run(self):
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            rospy.loginfo("Decision running")
            rate.sleep()





if __name__ == '__main__':
    rospy.init_node('DecisionNode')
    node = Decision()
    node.run()




