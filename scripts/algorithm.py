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

from geometry_msgs.msg import Twist
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs
#from decision_node.scripts import Condition
import math as m


class WayPoint():
    def __init__(self):
        self.goal_table = [
            ( 0.53,    0,      0),
            #(    0, 0.53, m.pi/2),
            (-0.53,    0,   m.pi),
            #(    0,-0.53,-m.pi/2),
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

    def execute(self, Condition):
        r = rospy.Rate(5) # change speed 5fps

        x,y,th = self.wayPoint.getWayPoint()
        
        rospy.loginfo("waypoint : {} {} {}".format(x,y,th))
        #self.setGoal(0,-0.5,3.1415)
        #self.setGoal(x,y,th)
        self.setGoal(x,y,th)


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
    rospy.init_node('BasicRun_unittest')
    node = BasicRun()
    node.run()











