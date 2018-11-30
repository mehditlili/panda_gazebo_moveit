#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "panda_move_group");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // setting up "planning groups" (JointModelGroup)
  static const std::string PLANNING_GROUP = "panda_arm";

  //setting up MoveGroup class 
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // for visualization purposes
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();

  // dont know if useful
  visual_tools.trigger();

  ///// planning a pose goal

  /*
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.2;
  target_pose1.position.z = 0.5;
  move_group.setPoseTarget(target_pose1);
  */


  ///// Planning joint space goal

  // pointer to current robot state
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

 // setting pose of joints
 /*
  joint_group_positions[0] = -0.610865;  // radians
  joint_group_positions[1] = -1.57;
  joint_group_positions[2] = 0.785398;
  joint_group_positions[3] = 3.14;
  joint_group_positions[4] = 0.610865;
  joint_group_positions[5]= 3.14;
  joint_group_positions[6]= -1.57;
  move_group.setJointValueTarget(joint_group_positions);
  
/*
  joint_group_positions[0] = 0;  // radians
  joint_group_positions[1] = 0;
  joint_group_positions[2] = 0;
  joint_group_positions[3] = 0;
  joint_group_positions[4] = 0;
  joint_group_positions[5]= -3.14;
  joint_group_positions[6]= 0;
  move_group.setJointValueTarget(joint_group_positions);
*/
  

  //call the planner to plan and visualize it
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  //moving to pose goal
  move_group.move();

  ros::shutdown();
  return 0;
}
