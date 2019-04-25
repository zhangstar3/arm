#!/usr/bin/env python
import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import  PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose
from copy import deepcopy
from math import *
from numpy  import *

b=1
a='%d'%b
object_id='object'+a


print(object_id)