import rospy,actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from trajectory_msgs.msg import *
from math import pi as PI,degrees,radians

from joint import Joint

class FollowController:
    def __init__(self,name):
        self.name=name
        rospy.init_node(self.name)
        self.rate=50.0

        #初始化
        self.arm0=Joint('joint0',0,1.5797,-1.5707,130,0,False)
        self.arm1=Joint('joint1',1,1.5797,-1.5707,130,0,False) 
        self.arm2=Joint('joint2',2,1.5797,-1.5707,130,0,False) 
        self.arm3=Joint('joint3',3,1.5797,-1.5707,130,0,False) 
        self.arm4=Joint('joint4',4,1.5797,-1.5707,130,0,False) 
        self.arm5=Joint('joint5',5,1.5797,-1.5707,130,0,False) 

        self.joints=[self.arm0,self.arm1,self.arm2,self.arm3,self.arm4,self.arm5]

        self.server=actionlib.SimpleActionServer('arm_controller/follow_joint_trajectory',FollowJointTrajectoryAction,
                                                execute_cb=self.actionCb,auto_start=False)
        
        self.server.start()
        rospy.spin()
        rospy.loginfo("Started FollowController")

    def actionCb(self,goal):
        rospy.loginfo(self.name+":Action goal recieved.")
        traj=goal.trajectory

        if(set(self.joints)!=set(traj.joint_names)):
            self.server.set_aborted(" does not match")
            rospy.loginfo("Extra joints in trajectory")
        
        if not traj.points:
            rospy.logger("trajectory empty")
            self.server.set_aborted("empty points")
            return

            #列举各种错误情况，到这里执行，还有几个未列举
        if self.executeTrajectory(traj):
            rospy.loginfo("Execute trajectory")
            rospy.logdebug(traj)
            try:
                indexes=[traj.joint_names.index(joint.name) for joint in self.joints]
            execpt ValueError as val:
                rospy.logerr("Invalid joint in trajetory")
                return False

            start=traj.header.stamp
            if start.secs==0 and start.nsecs==0:
                start=rospy.Time.now()
            
            r=rospy.Rate(self.rate)

            for point in traj.points:
                desired=[point.positions[k] for k in indexes]
                for i in indexes:
                    self.joints[i]. (desired[i])
                while rospy.Time.now()+rospy.Duration(0.01) <start:
                    rospy.sleep(0.01)
            return true
if __name__=='__main__':
    FollowController('follow_Controller')

