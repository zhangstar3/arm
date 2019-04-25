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
import types

GRIPPER_OPEN = [-0.015,-0.015]
GRIPPER_CLOSE = [-0.015,-0.015]
"""
 ros::Publisher plan_positions_pub = nh.advertise<sensor_msgs::JointState>("/plan/fake_robot_state", 100);
void pubMotionData(trajectory_msgs::JointTrajectory planData) {
  sensor_msgs::JointState fake_robot_state;
  fake_robot_state.header = planData.header;
  ros::Time init_time(0.0);
  for (int i = 0; i < planData.points.size(); i++) {
    fake_robot_state.header.stamp = init_time + planData.points[i].time_from_start;
    fake_robot_state.position = planData.points[i].positions;
    fake_robot_state.velocity = planData.points[i].velocities;
    fake_robot_state.effort = planData.points[i].accelerations;
    plan_positions_pub.publish(fake_robot_state);
  }
}
"""
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
        #manipulator = MoveGroupCommander('right_arm_controller')
        #endeffector = MoveGroupCommander('right_gripper_controller') 


        end_eff = manipulator.get_end_effector_link()
        #print("end_eff",end_eff)
        
        manipulator.set_goal_position_tolerance(0.001)
        manipulator.set_goal_orientation_tolerance(0.002)
              
        manipulator.allow_replanning(True)

        global_frame = manipulator.get_planning_frame()
        print("global_frame=",global_frame)
       
        manipulator.set_planning_time(2)

        #object1_id = 'object1'
        #object2_id = 'object2'
        #scene.remove_world_object(object1_id)
        #scene.remove_world_object(object2_id)

        for i in range(1,100):
            a='%d'%i
            object_id='object'+a
            scene.remove_world_object(object_id)
        rospy.sleep(0.7)

        obstacle1_id = 'obstacle1'
        obstacle2_id = 'obstacle2'
        scene.remove_world_object(obstacle1_id)
        scene.remove_world_object(obstacle2_id)
        rospy.sleep(0.5)


        obstacle1_size = [0.8, 0.3, 0.05]               
        obstacle1_pose = PoseStamped()
        obstacle1_pose.header.frame_id = global_frame
        obstacle1_pose.pose.position.x = 0.1
        obstacle1_pose.pose.position.y = 1.1
        obstacle1_pose.pose.position.z = 0.6
        obstacle1_pose.pose.orientation.w = 1.0   
        scene.add_box(obstacle1_id, obstacle1_pose, obstacle1_size)
        self.setColor(obstacle1_id, 0.8, 0.4, 0, 1.0)
        self.sendColors()

        obstacle2_size = [0.3, 0.3, 0.05]               
        obstacle2_pose = PoseStamped()
        obstacle2_pose.header.frame_id = global_frame
        obstacle2_pose.pose.position.x = -0.7
        obstacle2_pose.pose.position.y = 0.5
        obstacle2_pose.pose.position.z = 0.93
        obstacle2_pose.pose.orientation.w = 1.0   
        scene.add_box(obstacle2_id, obstacle2_pose, obstacle2_size)
        self.setColor(obstacle2_id, 0.8, 0.4, 0, 1.0)
        self.sendColors()

        ###################
        x=0.0
        y=0.0
        z=1.0
        theta=pi*0.29
        w1,x1,y1,z1=self.quaternion_mul(x,y,z,theta,0.701,0.701,0.0,0.0)

        print("w,x,y,z",w1,x1,y1,z1)
        ##################

      


        ####0
        manipulator.set_named_target('ready_pose')
        manipulator.go()
        rospy.sleep(0.1)
        print("=======  ok---0 =======")


        aims_x=0.4
        aims_y=1.2
        aims_z=0.87
        t=0
        wop_z =1.0

        for i in range(1,7):
            for k in range(1,5):
                a='%d'%(t+k)
                object_id='object'+a
                self.move_da(object_id,global_frame,scene,manipulator,end_eff,w1,x1,y1,z1,aims_x,aims_y,aims_z,0.16,wop_z)
                aims_x-=0.165
                
            if i%2==1:
                t=t+6
                a='%d'%(t)
                object_id='object'+a
                aims_x +=0.04
                self.move_da(object_id,global_frame,scene,manipulator,end_eff,w1,x1,y1,z1,aims_x,aims_y,aims_z,0.08,wop_z)

                aims_x=0.44
                aims_z += 0.045
                wop_z+=0.045
                t=t+1
                a='%d'%(t)
                object_id='object'+a
                self.move_da(object_id,global_frame,scene,manipulator,end_eff,w1,x1,y1,z1,aims_x,aims_y,aims_z,0.08,wop_z)
                aims_x-=0.125
            
            if i%2==0:
                t=t+5
                aims_x=0.4
                aims_z += 0.045
                wop_z+=0.045



        
        
        

        """

        print("=======  ok---1 =======")
        ####2
        waypoints1 = []
        start_pose = manipulator.get_current_pose(end_eff).pose
        wpose = deepcopy(start_pose)
        waypoints1.append(deepcopy(wpose))
       
        wpose.position.x -= 0.2
        wpose.position.y += 0.0
        wpose.position.z += 0.07
        waypoints1.append(deepcopy(wpose))
  
        wpose.position.x -= 0.2
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
        scene.remove_attached_object(end_eff, object1_id)
        rospy.sleep(0.5) 

        print("======  ok---2   =========")
        """



        manipulator.set_named_target('ready_pose')
        manipulator.go()
        rospy.sleep(1)
        print("=======  ok---all =======")


        moveit_commander.roscpp_shutdown()        
        moveit_commander.os._exit(0)


    def move_da(self,object_id,global_frame,scene,manipulator,end_eff,w1,x1,y1,z1,aims_x,aims_y,aims_z,long,wop_z):
        object_id=object_id
        a=0.04
        object_size = [long, 2*a, a] 
        object_pose = PoseStamped()
        object_pose.header.frame_id =global_frame              
        object_pose.pose.position.x = -0.6
        object_pose.pose.position.y = 0.5
        object_pose.pose.position.z = 0.98
        object_pose.pose.orientation.x=0.0
        object_pose.pose.orientation.y = 0.0
        object_pose.pose.orientation.z = 0.7071
        object_pose.pose.orientation.w = 0.7071       
        scene.add_box(object_id, object_pose, object_size)
        #rospy.sleep(0.05) 
        print("=========zhi1==========")

        ####1
        manipulator.set_start_state_to_current_state()
        target_pose = PoseStamped()
        target_pose.header.frame_id = global_frame
        target_pose.pose.position.x = -0.6
        target_pose.pose.position.y = 0.5
        target_pose.pose.position.z = 1.2
        target_pose.pose.orientation.x =0.659 #0.0428 #0.659
        target_pose.pose.orientation.y =-0.240  #0.7043 #-0.240
        target_pose.pose.orientation.z =-0.234 #-0.7084 #-0.234
        target_pose.pose.orientation.w =0.672 #0.0152  #0.672
        manipulator.set_pose_target(target_pose, end_eff)
        manipulator.go()
        rospy.sleep(0.1)
        scene.attach_box(end_eff, object_id, object_pose, object_size)
        rospy.sleep(0.1)

       

        print("=========zhi2==========")

        """
        ####2
        manipulator.set_start_state_to_current_state()
        target_pose = PoseStamped()
        target_pose.header.frame_id = global_frame
        target_pose.pose.position.x = 0.0
        target_pose.pose.position.y = 1.3
        target_pose.pose.position.z = 0.8
        target_pose.pose.orientation.x =0.659 #0.0428 #0.659
        target_pose.pose.orientation.y =-0.240  #0.7043 #-0.240
        target_pose.pose.orientation.z =-0.234 #-0.7084 #-0.234
        target_pose.pose.orientation.w =0.672 #0.0152  #0.672
        manipulator.set_pose_target(target_pose, end_eff)
        manipulator.go()
        """

        
        waypoints1 = []
        start_pose = manipulator.get_current_pose(end_eff).pose
        wpose = deepcopy(start_pose)
        waypoints1.append(deepcopy(wpose))
          
        wpose.position.z += 0.07
        waypoints1.append(deepcopy(wpose))  

        """
        wpose.position.z -= 0.2
        wpose.position.x = -0.2
        wpose.position.y = 0.9
        waypoints1.append(deepcopy(wpose)) 
        """


        wpose.position.x = aims_x
        wpose.position.y = aims_y
        wpose.position.z = wop_z #1.0
        wpose.orientation.x =x1  #0.0428 #0.659
        wpose.orientation.y =y1  #0.7043 #-0.240
        wpose.orientation.z =z1#-0.7084 #-0.234
        wpose.orientation.w =w1 #0.0152  #0.672
        waypoints1.append(deepcopy(wpose))

        wpose.position.z = aims_z
        waypoints1.append(deepcopy(wpose))
        

        (cartesian_plan, fraction) = manipulator.compute_cartesian_path (
                                        waypoints1,   # waypoint poses
                                        0.002,        # eef_step 1cm
                                        0.0,         # jump_threshold
                                        True)        # avoid_collisions

        manipulator.execute(cartesian_plan)
        rospy.sleep(0.1)
        print("=========zhi3==========")
        
        scene.remove_attached_object(end_eff, object_id)
        rospy.sleep(0.1)

        """
        manipulator.set_start_state_to_current_state()
        current_pose = manipulator.get_current_pose(end_eff).pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = global_frame
        target_pose.pose.position.x = current_pose.position.x
        target_pose.pose.position.y = current_pose.position.y
        target_pose.pose.position.z = current_pose.position.z+0.2
        target_pose.pose.orientation.x = current_pose.orientation.x 
        target_pose.pose.orientation.y = current_pose.orientation.y
        target_pose.pose.orientation.z = current_pose.orientation.z
        target_pose.pose.orientation.w = current_pose.orientation.w
        manipulator.set_pose_target(target_pose, end_eff)
        manipulator.go()
        rospy.sleep(0.1)
        
        """
        manipulator.set_named_target('ready_pose')
        manipulator.go()
        
        
        
       

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
    





    
