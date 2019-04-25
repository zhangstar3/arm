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
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        rospy.init_node('moveit_demo')
        
        # Construct the initial scene object
        scene = PlanningSceneInterface()
        
        # Create a scene publisher to push changes to the scene
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene)
        
        # Create a dictionary to hold object colors
        self.colors = dict()
        
        # Pause for the scene to get ready
        rospy.sleep(1)
                        
        # Initialize the move group for the right arm
        manipulator = MoveGroupCommander('arm')
        endeffector = MoveGroupCommander('armend')

        #manipulator = MoveGroupCommander('right_arm_controller')
        #endeffector = MoveGroupCommander('right_gripper_controller') 

        # Get the name of the end-effector link
        left_eef = manipulator.get_end_effector_link()

        #print("left_eef",left_eef)
        
        # Allow some leeway in position (meters) and orientation (radians)
        manipulator.set_goal_position_tolerance(0.02)
        manipulator.set_goal_orientation_tolerance(0.05)
       
        # Allow replanning to increase the odds of a solution
        manipulator.allow_replanning(True)

        
        left_reference_frame = manipulator.get_planning_frame()
        print("left_reference_frame=",left_reference_frame)

        # Allow 5 seconds per planning attempt
        manipulator.set_planning_time(5)

        object1_id = 'object1'
        obstacle1_id = 'obstacle1'
        # Remove leftover objects from a previous run
        scene.remove_world_object(object1_id)
        scene.remove_world_object(obstacle1_id)
        
        # Give the scene a chance to catch up
        rospy.sleep(1)
        
        # Start the arm in the "resting" pose stored in the SRDF file
        manipulator.set_named_target('home')
        manipulator.go()
        rospy.sleep(1)

        ###################
        x=1.0
        y=0.0
        z=0.0
        theta=pi/2
        w0=cos(theta/2)
        x0=x*sin(theta/2)
        y0=y*sin(theta/2)
        z0=z*sin(theta/2)
        w1,x1,y1,z1=self.quaternion_mul(w0,x0,y0,z0,-0.417,-0.603,0.291,0.613)
        print("w,x,y,z",w1,x1,y1,z1)
        ##################

        object1_size = [0.1, 0.08, 0.02]   

        object1_pose = PoseStamped()
        object1_pose.header.frame_id =left_reference_frame               #left_reference_frame 
        object1_pose.pose.position.x = 0.44308
        object1_pose.pose.position.y = 0.347721
        object1_pose.pose.position.z = 0.04

        object1_pose.pose.orientation.x=0.0
        object1_pose.pose.orientation.y = 0.0
        object1_pose.pose.orientation.z = 0.0
        object1_pose.pose.orientation.w = 1.0
        
        scene.add_box(object1_id, object1_pose, object1_size)

        obstacle1_size = [1.0, 0.4, 0.05]
        
        # Add a table top and two boxes to the scene
        
        obstacle1_pose = PoseStamped()
        obstacle1_pose.header.frame_id = left_reference_frame
        obstacle1_pose.pose.position.x = 0.0
        obstacle1_pose.pose.position.y = 1.0
        obstacle1_pose.pose.position.z = 0.4
        obstacle1_pose.pose.orientation.w = 1.0   
        #scene.add_box(obstacle1_id, obstacle1_pose, obstacle1_size)
       
        self.setColor(obstacle1_id, 0.8, 0.4, 0, 1.0)

        print("333333333333")
       
        self.setColor(object1_id, 0.8, 0, 0, 1.0)
        
        # Send the colors to the planning scene
        self.sendColors()   
        
        # Set the start state to the current state
        manipulator.set_start_state_to_current_state()
        # Set the target pose in between the boxes and above the table
        target_pose = PoseStamped()
        target_pose.header.frame_id = left_reference_frame
        target_pose.pose.position.x = 0.44308
        target_pose.pose.position.y = 0.347721
        target_pose.pose.position.z = 0.0439246
        
        target_pose.pose.orientation.x =0.0156737
        target_pose.pose.orientation.y =0.00239437
        target_pose.pose.orientation.z =0.108514
        target_pose.pose.orientation.w =0.993968
        
        
        # Set the target pose for the arm
        manipulator.set_pose_target(target_pose, left_eef)
        
        # Move the arm to the target pose (if possible)

        #manipulator.go()

        rospy.sleep(5)
        print("1111111111111")
        # Pause for a moment...
        
        scene.attach_box(left_eef, object1_id, object1_pose, object1_size)
        rospy.sleep(1)
        # Cartesian Paths
        waypoints1 = []
        start_pose = manipulator.get_current_pose(left_eef).pose
        #print("x,y,z",start_pose.position.x,start_pose.position.y,start_pose.position.z)
        wpose = deepcopy(start_pose)
        waypoints1.append(deepcopy(wpose))
       
        wpose.position.x -= 0.05
        wpose.position.y -= 0.5
        wpose.position.z += 0.07
        waypoints1.append(deepcopy(wpose))
  
        #wpose.position.x -= 0.05
        #wpose.position.y -= 0.5
        #wpose.position.z -= 0.07
        #waypoints1.append(deepcopy(wpose))

        (cartesian_plan, fraction) = manipulator.compute_cartesian_path (
                                        waypoints1,   # waypoint poses
                                        0.01,        # eef_step 1cm
                                        0.0,         # jump_threshold
                                        True)        # avoid_collisions

         

        #manipulator.execute(cartesian_plan)
        print("222222222222")
        rospy.sleep(1.5)
        scene.remove_attached_object(left_eef, object1_id) 
        # Exit MoveIt cleanly
        
        waypoints2 = []
        start_pose = manipulator.get_current_pose(left_eef).pose
        wpose = deepcopy(start_pose)
        waypoints2.append(deepcopy(wpose))
        wpose.position.z += 0.07
        waypoints2.append(deepcopy(wpose))

        wpose.position.x += 0.1
        wpose.position.y += 0.21
        waypoints2.append(deepcopy(wpose))

        wpose.position.z -= 0.07
        waypoints2.append(deepcopy(wpose))

        (cartesian_plan, fraction) = manipulator.compute_cartesian_path (
                                        waypoints2,   # waypoint poses
                                        0.01,        # eef_step 1cm
                                        0.0,         # jump_threshold
                                        True)        # avoid_collisions

         

        #manipulator.execute(cartesian_plan)
        rospy.sleep(2)
        
        moveit_commander.roscpp_shutdown()
        
        # Exit the script        
        moveit_commander.os._exit(0)


    def quaternion_mul(self,a1,b1,c1,d1,a2,b2,c2,d2):
        w=a1*a2-b1*b2-c1*c2-d1*d2
        x=a1*b2+b1*a2+c1*d2-d1*c2
        y=a1*c2-b1*d2+c1*a2+d1*b2
        z=a1*d2+b1*c2-c1*b2+d1*a2
        list1=[w,x,y,z]
        return list1    

    # Set the color of an object
    def setColor(self, name, r, g, b, a = 0.9):
        # Initialize a MoveIt color object
        color = ObjectColor()
        
        # Set the id to the name given as an argument
        color.id = name
        
        # Set the rgb and alpha values given as input
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        
        # Update the global color dictionary
        self.colors[name] = color

    # Actually send the colors to MoveIt!
    def sendColors(self):
        # Initialize a planning scene object
        p = PlanningScene()

        # Need to publish a planning scene diff        
        p.is_diff = True
        
        # Append the colors from the global color dictionary 
        for color in self.colors.values():
            p.object_colors.append(color)
        
        # Publish the scene diff
        self.scene_pub.publish(p)

if __name__ == "__main__":
    try:
        MoveItDemo()
    except KeyboardInterrupt:
        raise
    
    
    
