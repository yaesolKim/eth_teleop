#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <iostream>
#include "std_msgs/String.h"

geometry_msgs::Pose p_c;
bool new_p;
bool isStatic;

// subscribe contact predicted position from Unreal
void haptic_callback(geometry_msgs::PoseStamped p) {

  if( p_c.position.x != p.pose.position.x * (0.1) ) {
    //p_c = p.pose;

    p_c.position.x = p.pose.position.x* (0.1);
    p_c.position.y = p.pose.position.y* (-0.1);
    p_c.position.z = p.pose.position.z* (0.1);

    if (p.header.frame_id.c_str() == "s") {
        ROS_INFO("STATIC: frame id: %s", p.header.frame_id.c_str());
        isStatic = 1;
    }
    else if (p.header.frame_id.c_str() == "d"){
        ROS_INFO("DYNAMIC: frame id: %s", p.header.frame_id.c_str());
        isStatic = 0;
    }
    //ROS_INFO("move to %f, %f, %f", p_c.position.x, p_c.position.y, p_c.position.z );

    new_p = 1;
  }

}

/*
void haptic_callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}
*/

int main(int argc, char** argv)
{
  ros::init(argc, argv, "haptic");

  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Subscriber p_c_sub = node_handle.subscribe("/cp_haptic", 1, haptic_callback);

  static const std::string PLANNING_GROUP = "panda_arm";
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  geometry_msgs::PoseStamped current_pose = move_group_interface.getCurrentPose("panda_link8");
  std::cout << std::endl <<"pos: " << current_pose.pose.position.x << ", " << current_pose.pose.position.y << ", " << current_pose.pose.position.z << std::endl;
  std::cout <<"ori: " << current_pose.pose.orientation.x << ", " << current_pose.pose.orientation.y << ", " << current_pose.pose.orientation.z << ", " << current_pose.pose.orientation.w << std::endl;

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to test motion");
  geometry_msgs::Pose start_pose = current_pose.pose;
  start_pose.position.x += 0.2;
  start_pose.position.y += 0.2;
  start_pose.position.z += 0.0;
  move_group_interface.setApproximateJointValueTarget(start_pose, "panda_link8");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  move_group_interface.move();

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start haptic demo");
  p_c = move_group_interface.getCurrentPose("panda_link8").pose;
  new_p = 0;
  isStatic = 1;

  while(ros::ok) {

    if(new_p) {
        move_group_interface.setApproximateJointValueTarget(p_c, "panda_link8");
        success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        move_group_interface.move();

        //if (isStatic) -> cartesian + hold configuration
        //else -> cartesian + impedance

        new_p = 0;
    }
/* else-> nothing
    else {
        visual_tools.prompt("Press 'next'");
    }
*/
  }



/*
  // Planning with Path Constraints
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Path constraints can easily be specified for a link on the robot.
  // Let's specify a path constraint and a pose goal for our group.
  // First define the path constraint.
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "panda_link8";
  ocm.header.frame_id = "panda_link0";
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;

  // Now, set it as the path constraint for the group.
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group_interface.setPathConstraints(test_constraints);

  success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  visual_tools.publishText(text_pose, "move test", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' to test the right arm move");
  move_group_interface.move();


  // When done with the path constraint be sure to clear it.
  move_group_interface.clearPathConstraints();
*/
  /* Wait for MoveGroup to receive and process the attached collision object message */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to finish");

  // END_TUTORIAL

  ros::shutdown();
  return 0;
}
