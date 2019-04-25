#!/usr/bin/env python

"""
    moveit_ik_demo.py - Version 0.1 2014-01-14
    
    Use inverse kinemtatics to move the end effector to a specified pose
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import  PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose
from copy import deepcopy
from math import *
from numpy  import *


class MoveItDemo:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)   
    
        rospy.init_node('moveit_demo') 
          
        scene = PlanningSceneInterface() 
  
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene)

        self.colors = dict()                     
        manipulator = MoveGroupCommander('arm')

        endeffector = MoveGroupCommander('armend')  

        end_eff = manipulator.get_end_effector_link()

        manipulator.set_goal_position_tolerance(0.02)

        manipulator.set_goal_orientation_tolerance(0.05)
 
        manipulator.allow_replanning(True)

        global_frame = manipulator.get_planning_frame()

        manipulator.set_planning_time(5)

        x=1.0
        y=0.0
        z=0.0
        theta=pi/2
        w1,x1,y1,z1=self.quaternion_mul(x,y,z,theta,-0.477982,0.751584,-0.387842,-0.237134)
        print("w,x,y,z",w1,x1,y1,z1)

        waypoints1 = []
        start_pose = manipulator.get_current_pose(end_eff).pose
        wpose = deepcopy(start_pose)
        waypoints1.append(deepcopy(wpose)) 
        wpose.position.x+= 0.1
        wpose.position.y += 0.1
        waypoints1.append(deepcopy(wpose))
        wpose.position.x = -0.1
        wpose.position.y = 0.2
        wpose.position.z = 1.2
        wpose.orientation.x =z1
        wpose.orientation.y =y1
        wpose.orientation.z =-x1
        wpose.orientation.w =w1
        waypoints1.append(deepcopy(wpose))    
        (cartesian_plan, fraction) = manipulator.compute_cartesian_path (waypoints1,   # waypoint poses
                                        0.01,        # eef_step 1cm
                                        0.0,         # jump_threshold
                                        True)        # avoid_collisions
        manipulator.execute(cartesian_plan)
        rospy.sleep(1.5)
      
   
        moveit_commander.roscpp_shutdown()        
        moveit_commander.os._exit(0)


    def quaternion_mul(self,x,y,z,theta,a2,b2,c2,d2):
        a1=cos(theta/2)
        b1=x*sin(theta/2)
        c1=y*sin(theta/2)
        d1=z*sin(theta/2)
        w=a1*a2-b1*b2-c1*c2-d1*d2
        x=a1*b2+b1*a2+c1*d2-d1*c2
        y=a1*c2-b1*d2+c1*a2+d1*b2
        z=a1*d2+b1*c2-c1*b2+d1*a2
        list1=[w,x,y,z]
        return list1  
##################

        
       



if __name__ == "__main__":
    try:
        MoveItDemo()
    except KeyboardInterrupt:
        raise
    
    
    
