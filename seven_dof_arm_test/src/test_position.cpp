#include<moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
int main(int argc, char **argv)

{

  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle; 
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Rate rate(10);
  moveit::planning_interface::MoveGroupInterface group("arm");

  // 设置机器人终端的目标位置
  group.setGoalPositionTolerance(0.05);
  group.setGoalOrientationTolerance(0.08);
  geometry_msgs::Pose target_pose1;
  double a[7]={ 
    //0.3,0,0.2,0 , 0,0,1
    //0.347672,0.0535878,0.1597,-0.0808603,-0.683757,0.719828,0.0882383 //最开始的机械臂home
    //0.494523,-0.0103116,0.0454806,0,0,0,1 //新机械臂模型home
    0.3,0.1,0.04,0,0,0,1  //物块
  };//x y z  xyzw
  target_pose1.orientation.w =a[6];//-0.237134;//-0.06789;//0.726282;
  target_pose1.orientation.x =a[3];//-0.477982;//-0.076335;//4.04423e-07;
  target_pose1.orientation.y =a[4];//0.751584;//-0.70385;//-0.687396;
  target_pose1.orientation.z =a[5];//-0.387842;//0.70385;//4.81813e-07;
//0;;//;//
  target_pose1.position.x    =a[0];//0.239637;// 0.6398;//0.0261186;
  target_pose1.position.y    =a[1];//0.331536;// -0.006899;//4.50972e-07;
  target_pose1.position.z    =a[2];//0.15593;// 0.40611;//0.573659;

  group.setPoseTarget(target_pose1);

  // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);



  //ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");   

  //让机械臂按照规划的轨迹开始运动。
  
  if(success)
    group.execute(my_plan); //不执行了

  sleep(2);

  geometry_msgs::Pose target_pose2;
  double b[7]={ 
    //0.44308,0.347721,0.0439246,0.0156737,0.00239437,0.108514,0.993968 //新机械臂模型ready
    0.472387,0.24218,0.0416062,-0.00355626,-0.00849876,0.0506212,0.998765 //pose_1位置
    //0.429021,0.234403,0.144769,-0.0207877,0.0144266,-0.0130599,0.99595
  };//x y z  xyzw
  target_pose2.orientation.w =b[6];
  target_pose2.orientation.x =b[3];
  target_pose2.orientation.y =b[4];
  target_pose2.orientation.z =b[5];

  target_pose2.position.x    =b[0];
  target_pose2.position.y    =b[1];
  target_pose2.position.z    =b[2];
  group.setPoseTarget(target_pose2);
  group.plan(my_plan);
  group.execute(my_plan);
  

 


  rate.sleep();
  ros::shutdown(); 
  return 0;

}