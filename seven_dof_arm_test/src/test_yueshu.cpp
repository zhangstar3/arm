#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
 
int main(int argc, char **argv)
{
   ros::init(argc, argv, "move_group_interface_tutorial");
   ros::NodeHandle node_handle; 
   ros::AsyncSpinner spinner(1);
   spinner.start();
   moveit::planning_interface::MoveGroupInterface group("arm");
   moveit::planning_interface::MoveItErrorCode success;
 
   //设置初始位置
   group.setStartState(*group.getCurrentState());
 
   //设置约束
   moveit_msgs::Constraints endEffector_constraints;
   moveit_msgs::OrientationConstraint ocm;
   ocm.link_name = "Empty_Link_5";//需要约束的链接
   ocm.header.frame_id = "base_link";//基坐标系
   //四元数约束
   //ocm.orientation.w = 1.0;
   //欧拉角约束
   ocm.absolute_x_axis_tolerance = 2*3.14;
   ocm.absolute_y_axis_tolerance = 2*3.14;
   ocm.absolute_z_axis_tolerance = 2*3.14;
   ocm.weight = 1.0;//此限制权重
   endEffector_constraints.orientation_constraints.push_back(ocm);//加入限制列表
   group.setPathConstraints(endEffector_constraints);//设置约束
	
   //设置抓取目标点 
   geometry_msgs::Pose target_pose1;
   //末端姿态四元数
  target_pose1.orientation.w =-0.06789;//0.726282;
  target_pose1.orientation.x =0.076335;//4.04423e-07;
  target_pose1.orientation.y =-0.70385;//-0.687396;
  target_pose1.orientation.z =0.70385;//4.81813e-07;
//0;
  target_pose1.position.x   = 0.6398;//0.0261186;
  target_pose1.position.y   = -0.006899;//4.50972e-07;
  target_pose1.position.z   = 0.40611;//0.573659;
 
   //进行运动规划
   group.setPoseTarget(target_pose1);
   moveit::planning_interface::MoveGroupInterface::Plan plan;
   success = group.plan(plan);
   group.clearPathConstraints();//清除约束
   
   //运动规划输出
   ROS_INFO("Visualizing plan (stateCatch pose) %s",success == moveit_msgs::MoveItErrorCodes::SUCCESS ? "SUCCESS" : "FAILED");
   if (success  == moveit_msgs::MoveItErrorCodes::SUCCESS)  group.execute(plan);
 
   ros::shutdown(); 
   return 0;
}
