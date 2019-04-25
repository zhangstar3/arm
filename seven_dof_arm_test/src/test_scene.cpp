
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include<moveit/move_group_interface/move_group_interface.h>
// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>



int main(int argc, char** argv)
{
  ros::init(argc, argv, "planning_scene_ros_api_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Rate rate(10);
  ros::NodeHandle node_handle;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit_msgs::CollisionObject collision_object;
  moveit::planning_interface::MoveGroupInterface group("arm");
  collision_object.header.frame_id = group.getPlanningFrame();
  collision_object.id = "box1";// 该对象的ID用于标识它。

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.1;
  primitive.dimensions[1] = 0.08;
  primitive.dimensions[2] = 0.01;

  //为盒子定义姿势（相对于frame_id）
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.47308;
  box_pose.position.y = 0.242721;
  box_pose.position.z = 0.03;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // 现在，让我们添加碰撞对象到世界中
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  group.attachObject(collision_object.id);

  geometry_msgs::Pose target_pose1;
  double a[7]={ 
    //0.3,0,0.2,0 , 0,0,1
    //0.347672,0.0535878,0.1597,-0.0808603,-0.683757,0.719828,0.0882383 //最开始的机械臂home
    0.494523,-0.0103116,0.0454806,0,0,0,1 //新机械臂模型home
    //0.3,0.1,0.04,0,0,0,1  //物块
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
   //group.setNamedTarget("home");
  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  // moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
  // group.plan(my_plan);
  //group.execute(my_plan);
  group.move();
  sleep(5);
  // group.setRandomTarget();
  // group.move();
  // sleep(5);

  group.setStartStateToCurrentState();
  geometry_msgs::Pose target_pose3 = group.getCurrentPose().pose;

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(target_pose3);

  target_pose3.position.z += 0.2;
  waypoints.push_back(target_pose3);  // down

  target_pose3.position.y -= 0.2;
  waypoints.push_back(target_pose3);  // right

  target_pose3.position.z += 0.2;
  target_pose3.position.y += 0.2;
  target_pose3.position.x -= 0.2;
  waypoints.push_back(target_pose3);  // up and left

  // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
  // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
  // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
  group.setMaxVelocityScalingFactor(0.1);

  // We want the Cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in Cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
  // Warning - disabling the jump threshold while operating real hardware can cause
  // large unpredictable motions of redundant joints and could be a safety issue
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.00;
  const double eef_step = 0.01;
  //double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  //ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);




  rate.sleep();
  ros::shutdown();
  return 0;
}