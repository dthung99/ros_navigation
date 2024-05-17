// The code was adapted from https://github.com/moveit/moveit_tutorials/blob/melodic-devel/doc/move_group_interface/src/move_group_interface_tutorial.cpp
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

namespace rvt = rviz_visual_tools;

// Declare some parameters
// Some moveit parameters
double planning_time_limit = 10.0; // Time limit for path planning
moveit_msgs::RobotTrajectory trajectory; // Trajectory
const double jump_threshold = 0.0; // No large robot jump in joint space > 0.01 radian
double fraction; // How well the end effector follow the waypoints
bool success; // Variable to check if planning is success
moveit::planning_interface::MoveGroupInterface::Plan my_plan; // Call the planner to compute the plan and visualize it.
Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity(); // position in 3D for title
Eigen::Isometry3d position = Eigen::Isometry3d::Identity(); // position in 3D for visualization
const std::string& brain_mesh_file = "file:///home/rosbox/catkin_ws/src/needle_path_simulation/src/brain_model.dae";

void move_through_list_of_points(std::vector<geometry_msgs::Pose>& waypoints, 
                                 moveit::planning_interface::MoveGroupInterface& move_group,
                                 const robot_state::JointModelGroup* joint_model_group,
                                 moveit_visual_tools::MoveItVisualTools& visual_tools,
                                 geometry_msgs::Pose brain_model_pose,
                                 double velociy_scaling = 1.0,
                                 double eef_step = 0.001, // step to discretize in task space
                                 std::string action = "default action",
                                 bool show_brain = true)
  {    
  move_group.setMaxVelocityScalingFactor(velociy_scaling); // change the movement speed
  visual_tools.prompt("Press 'next' to " + action); // Interactive UI
  // Plan the trajectory;
  fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  my_plan.trajectory_ = trajectory;
  // Visualization
  visual_tools.deleteAllMarkers();
  position.translation().x() = waypoints.back().position.x;
  position.translation().y() = waypoints.back().position.y;
  position.translation().z() = waypoints.back().position.z;
  visual_tools.publishSphere(position.translation(), rvt::PURPLE, rvt::XXLARGE);
  visual_tools.publishText(text_pose, "Path to " + action, rvt::PURPLE, rvt::XXXXLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group, rvt::TRANSLUCENT_LIGHT);
  if (show_brain) {visual_tools.publishMesh(brain_model_pose, brain_mesh_file, rvt::PURPLE, 0.001, "mesh", 1);} // Add brain model
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' to accept the planning and " + action); // Interactive UI
  const std::string& text_4 = action.c_str();
  ROS_INFO_NAMED("tutorial", "%s", action.c_str()); // Interactive UI
  move_group.execute(my_plan); // Move the robot
  const std::string& text_5 = action + ": Success!!!";
  ROS_INFO_NAMED("tutorial", "%s: Success!!!", action.c_str()); // Interactive UI
}


int main(int argc, char** argv)
{
  // ************************************
  // Set up stuff
  // ************************************
  // ************************************
  // Declare some parameters
  // Some moveit parameters
  std::vector<geometry_msgs::Pose> waypoints; // List of via point in task space
  // Initial and target pose
  geometry_msgs::Pose initial_pose;
  geometry_msgs::Pose target_pose;
  geometry_msgs::Pose current_pose;
  // Define a pose for the brain_model (specified relative to frame_id)
  geometry_msgs::Pose brain_model_pose;
  brain_model_pose.orientation.w = 1.0;
  brain_model_pose.position.x = 0.0;
  brain_model_pose.position.y = 1.3;
  brain_model_pose.position.z = 1.0;

  // ************************************
  // Initiate ROS
  ros::init(argc, argv, "move_group_interface_control_node");
  ros::NodeHandle node_handle;
  // Get user parameters
  ros::param::get("/initial_pose/x", initial_pose.position.x);
  ros::param::get("/initial_pose/y", initial_pose.position.y);
  ros::param::get("/initial_pose/z", initial_pose.position.z);
  ros::param::get("/target_pose/x", target_pose.position.x);
  ros::param::get("/target_pose/y", target_pose.position.y);
  ros::param::get("/target_pose/z", target_pose.position.z);

  ros::param::get("/quaternion/x", initial_pose.orientation.x);
  ros::param::get("/quaternion/y", initial_pose.orientation.y);
  ros::param::get("/quaternion/z", initial_pose.orientation.z);
  ros::param::get("/quaternion/w", initial_pose.orientation.w);
  ros::param::get("/quaternion/x", target_pose.orientation.x);
  ros::param::get("/quaternion/y", target_pose.orientation.y);
  ros::param::get("/quaternion/z", target_pose.orientation.z);
  ros::param::get("/quaternion/w", target_pose.orientation.w);

  // Set single threading
  ros::AsyncSpinner spinner(1);
  spinner.start();
  // Setup the move group
  static const std::string PLANNING_GROUP = "robot_arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // Visualization with the package MoveItVisualTools
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();
  // Remote control allows users to step through a high level script
  visual_tools.loadRemoteControl();
  // via buttons and keyboard shortcuts in RViz
  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  text_pose.translation().z() = 1.75;
  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();
  // ************************************
  // Getting Basic Information
  // Print the name of the reference frame, end-effector link, planning group
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));
  // Set the planning time limit
  move_group.setPlanningTime(planning_time_limit);

  // ************************************
  // Control the robot
  // ************************************

  // ************************************
  // Move to initial pose
  waypoints.resize(0);
  waypoints.push_back(move_group.getCurrentPose().pose);
  waypoints.push_back(initial_pose);
  move_through_list_of_points(waypoints, move_group, joint_model_group, visual_tools, brain_model_pose, 1.0, 0.01, "Move to Start Pose", false);

  // ************************************
  // Add Brain Model to the environment
  visual_tools.prompt("Press 'next' to add brain model");
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Add brain model", rvt::PURPLE, rvt::XXXXLARGE);
  visual_tools.publishMesh(brain_model_pose, brain_mesh_file, rvt::PURPLE, 0.001, "mesh", 1); // Add brain model
  ROS_INFO_NAMED("tutorial", "Added brain model into the workspace"); // Show text in RViz of status 
  visual_tools.trigger(); // Sending visualization

  // ************************************
  // Set path constraints to make the needle point mantain desired orientation
  // Declare the constraints
  moveit_msgs::OrientationConstraint Needle_ocm;
  Needle_ocm.link_name = move_group.getEndEffectorLink();
  Needle_ocm.header.frame_id = move_group.getPlanningFrame();
  Needle_ocm.orientation = initial_pose.orientation;
  Needle_ocm.absolute_x_axis_tolerance = 0.01;
  Needle_ocm.absolute_y_axis_tolerance = 0.01;
  Needle_ocm.absolute_z_axis_tolerance = 0.01;
  Needle_ocm.weight = 1.0;
  // Now, set it as the path constraint for the group.
  moveit_msgs::Constraints Needle_constraints;
  Needle_constraints.orientation_constraints.push_back(Needle_ocm);
  move_group.setPathConstraints(Needle_constraints);

  // ************************************
  // Insert the needle
  waypoints.resize(0);
  waypoints.push_back(move_group.getCurrentPose().pose);
  waypoints.push_back(target_pose);
  move_through_list_of_points(waypoints, move_group, joint_model_group, visual_tools, brain_model_pose, 1.0, 0.001, "Insert the Needle");
  // ************************************
  // Remove the needle
  waypoints.resize(0);
  waypoints.push_back(move_group.getCurrentPose().pose);
  waypoints.push_back(initial_pose);
  move_through_list_of_points(waypoints, move_group, joint_model_group, visual_tools, brain_model_pose, 1.0, 0.001, "Remove the Needle");

  // ************************************
  // Ending the path planning
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Finished", rvt::PURPLE, rvt::XXXXLARGE);
  move_group.clearPathConstraints(); // Clear Path constraint
  visual_tools.trigger();
  ros::shutdown(); // Shut down ros
  return 0;
}