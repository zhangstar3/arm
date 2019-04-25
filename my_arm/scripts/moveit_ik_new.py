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

GRIPPER_OPEN = [-0.015,-0.015]
GRIPPER_CLOSE = [-0.015,-0.015]

class MoveItDemo:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)        
        rospy.init_node('moveit_demo')             
        scene = PlanningSceneInterface()    
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene)
        self.colors = dict()
        rospy.sleep(1)
                        
        manipulator = MoveGroupCommander('manipulator')
        endeffector = MoveGroupCommander('endeffector')  

        end_eff = manipulator.get_end_effector_link()
        #print("end_eff",end_eff)
        
        manipulator.set_goal_position_tolerance(0.02)
        manipulator.set_goal_orientation_tolerance(0.05)
              
        manipulator.allow_replanning(True)

        global_frame = manipulator.get_planning_frame()
        print("global_frame=",global_frame)
       
        manipulator.set_planning_time(5)

        object1_id = 'object1'
        obstacle1_id = 'obstacle1'
        obstacle2_id = 'obstacle2'
        
        scene.remove_world_object(object1_id)
        scene.remove_world_object(obstacle1_id)
        scene.remove_world_object(obstacle2_id)
        
        rospy.sleep(1.0)
        

        ###################
        x=1.0
        y=0.0
        z=0.0
        theta=pi/2
        w1,x1,y1,z1=self.quaternion_mul(x,y,z,theta,-0.417,-0.603,0.291,0.613)

        print("w,x,y,z",w1,x1,y1,z1)
        ##################

        object1_size = [0.1, 0.1, 0.01]   
        object1_pose = PoseStamped()
        object1_pose.header.frame_id =global_frame              
        object1_pose.pose.position.x = 0.3
        object1_pose.pose.position.y = 1.0
        object1_pose.pose.position.z = 0.5
        object1_pose.pose.orientation.x=0.0
        object1_pose.pose.orientation.y = 0.0
        object1_pose.pose.orientation.z = 0.0
        object1_pose.pose.orientation.w = 1.0       
        scene.add_box(object1_id, object1_pose, object1_size)
        self.setColor(object1_id, 0.8, 0, 0, 1.0)
        self.sendColors()

        obstacle1_size = [1.0, 0.4, 0.05]               
        obstacle1_pose = PoseStamped()
        obstacle1_pose.header.frame_id = global_frame
        obstacle1_pose.pose.position.x = 0.0
        obstacle1_pose.pose.position.y = 1.0
        obstacle1_pose.pose.position.z = 0.45
        obstacle1_pose.pose.orientation.w = 1.0   
        scene.add_box(obstacle1_id, obstacle1_pose, obstacle1_size)
        self.setColor(obstacle1_id, 0.8, 0.4, 0, 1.0)
        self.sendColors()

        obstacle2_size = [0.05, 0.4, 0.2]               
        obstacle2_pose = PoseStamped()
        obstacle2_pose.header.frame_id = global_frame
        obstacle2_pose.pose.position.x = 0.0
        obstacle2_pose.pose.position.y = 1.0
        obstacle2_pose.pose.position.z = 0.55
        obstacle2_pose.pose.orientation.w = 1.0   
        #scene.add_box(obstacle2_id, obstacle2_pose, obstacle2_size)
        #self.setColor(obstacle2_id, 0.8, 0.4, 0, 1.0)
        #self.sendColors()


       
       
        manipulator.set_start_state_to_current_state()
        target_pose = PoseStamped()
        target_pose.header.frame_id = global_frame
        target_pose.pose.position.x = 0.0
        target_pose.pose.position.y = 0.7
        target_pose.pose.position.z = 1.0
        target_pose.pose.orientation.x =0.659 #0.0428 #x
        target_pose.pose.orientation.y =-0.240  #0.7043
        target_pose.pose.orientation.z =-0.234 #-0.7084
        target_pose.pose.orientation.w =0.672 #0.0152
        manipulator.set_pose_target(target_pose, end_eff)
        manipulator.go()
        rospy.sleep(0.3)
       
       
        ####1
        manipulator.set_start_state_to_current_state()
        target_pose = PoseStamped()
        target_pose.header.frame_id = global_frame
        target_pose.pose.position.x = 0.3
        target_pose.pose.position.y = 1.0
        target_pose.pose.position.z = 0.7
        target_pose.pose.orientation.x =0.659 #0.0428 #x
        target_pose.pose.orientation.y =-0.240  #0.7043
        target_pose.pose.orientation.z =-0.234 #-0.7084
        target_pose.pose.orientation.w =0.672 #0.0152
        manipulator.set_pose_target(target_pose, end_eff)
        manipulator.go()
        rospy.sleep(0.3)
       
        scene.attach_box(end_eff, object1_id, object1_pose, object1_size)
        rospy.sleep(0.3)



        manipulator.set_start_state_to_current_state()
        target_pose = PoseStamped()
        target_pose.header.frame_id = global_frame
        target_pose.pose.position.x = -0.3
        target_pose.pose.position.y = 1.0
        target_pose.pose.position.z = 0.7
        target_pose.pose.orientation.x =0.659 #0.0428 #x
        target_pose.pose.orientation.y =-0.240  #0.7043
        target_pose.pose.orientation.z =-0.234 #-0.7084
        target_pose.pose.orientation.w =0.672 #0.0152
        manipulator.set_pose_target(target_pose, end_eff)
        manipulator.go()
        rospy.sleep(0.3)
        scene.remove_attached_object(end_eff, object1_id) 
        rospy.sleep(0.3)
         
        ####0
        manipulator.set_named_target('ready_pose')
        manipulator.go()
        rospy.sleep(1)
        print("=======  ok---0 =======")
        
    
        """
        print("=======  ok---1 =======")
        ####2
        waypoints1 = []
        start_pose = manipulator.get_current_pose(end_eff).pose
        wpose = deepcopy(start_pose)
        waypoints1.append(deepcopy(wpose))
       
        wpose.position.x -= 1.0
        wpose.position.y -= 0.3
        wpose.position.z -= 1.0
        waypoints1.append(deepcopy(wpose))
        
     
        wpose.position.x -= 0.5
        wpose.position.y += 0.0
        wpose.position.z -= 0.07
        waypoints1.append(deepcopy(wpose))
       
        (cartesian_plan, fraction) = manipulator.compute_cartesian_path (
                                        waypoints1,   # waypoint poses
                                        0.01,        # eef_step 1cm
                                        0.0,         # jump_threshold
                                        True)        # avoid_collisions

        manipulator.execute(cartesian_plan)
        rospy.sleep(1.5)
        #scene.remove_attached_object(end_eff, object1_id) 
        """
      




        """
        ####3
        waypoints2 = []
        start_pose = manipulator.get_current_pose(end_eff).pose
        wpose = deepcopy(start_pose)
        waypoints2.append(deepcopy(wpose))
        wpose.position.z += 0.07
        waypoints2.append(deepcopy(wpose))

        wpose.position.x += 0.1
        wpose.position.y -= 0.21
        waypoints2.append(deepcopy(wpose))

        wpose.position.z -= 0.07
        waypoints2.append(deepcopy(wpose))

        (cartesian_plan, fraction) = manipulator.compute_cartesian_path (
                                        waypoints2,   # waypoint poses
                                        0.01,        # eef_step 1cm
                                        0.0,         # jump_threshold
                                        True)        # avoid_collisions  
        manipulator.execute(cartesian_plan)
        rospy.sleep(2)
        print("=======  ok---3 =======")
        """


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

    
    def setColor(self, name, r, g, b, a = 0.9):
        
        color = ObjectColor()
        color.id = name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        self.colors[name] = color

 
    def sendColors(self):
        p = PlanningScene()
        p.is_diff = True
        for color in self.colors.values():
            p.object_colors.append(color)
        self.scene_pub.publish(p)

if __name__ == "__main__":
    try:
        MoveItDemo()
    except KeyboardInterrupt:
        raise
    
    
    
