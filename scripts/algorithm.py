#!/usr/bin/env python
# -*- coding: utf-8 -*-

## Multiarrayのpub
## https://asukiaaa.blogspot.com/2021/07/rostopic-pub-float32multiarray.html
## rostopic pub -r 1 Info_enemy std_msgs/Float32MultiArray "data: [20.0, 20.3]"

from decision_node.scripts.decision_node import EnemyAttitudeIdx
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
from decision_node.scripts import EnemyAttitudeIdx















