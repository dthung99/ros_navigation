// The code was adapted from https://github.com/moveit/moveit_tutorials/blob/melodic-devel/doc/move_group_interface/src/move_group_interface_tutorial.cpp
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// IGTlink msg
#include <ros_igtl_bridge/igtlpoint.h>
#include <ros_igtl_bridge/igtlpolydata.h>
#include <ros_igtl_bridge/igtlpointcloud.h>
#include <sensor_msgs/PointCloud.h>

namespace rvt = rviz_visual_tools;

// Declare some parameters
// Some moveit parameters
tf2::Vector3 needle_orientation_in_end_effector_frame(0.0,0.0,1.0); // Orientation of the needle wrt end effector frame
geometry_msgs::Pose initial_pose; // Initial and target pose
geometry_msgs::Pose target_pose; // Initial and target pose
double planning_time_limit = 10.0; // Time limit for path planning
moveit_msgs::RobotTrajectory trajectory; // Trajectory
double jump_threshold = 0.0; // No large robot jump in joint space > 0.01 radian
double fraction; // How well the end effector follow the waypoints
bool success = true; // Variable to check if planning is success
moveit::planning_interface::MoveGroupInterface::Plan my_plan; // Call the planner to compute the plan and visualize it.
Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity(); // position in 3D for title
const std::string& brain_mesh_file = "file:///home/rosbox/catkin_ws/src/needle_path_simulation/src/brain_model.dae";
// Pointer to control moveit
moveit_visual_tools::MoveItVisualTools* visual_tools_Ptr = nullptr;
moveit::planning_interface::MoveGroupInterface* move_group_Ptr = nullptr;
// Variable to communicate with 3D slicer
std::string entry_point_name = "entry_point"; // "entry_point_name";
std::string target_point_name = "target_point"; // "target_point_name";
double slicer_scale = 0.001; // The data in 3D slicer will be multiplied with the scale, 1mm -> 0.001
sensor_msgs::PointCloud brain_point_cloud; // DELETE later
ros::Publisher brain_point_cloud_pub; // DELETE later


// // ************************************************************************************************************
// // ************************************************************************************************************
// // Function place holder
// void set_needle_orientation_for_start_and_end_pose(geometry_msgs::Pose& initial_pose, geometry_msgs::Pose& target_pose);

// void set_constraint_from_initial_orientation(geometry_msgs::Pose& initial_pose,
//                                              moveit::planning_interface::MoveGroupInterface& move_group);

// void show_friendly_GUI_text(moveit_visual_tools::MoveItVisualTools& visual_tools, std::string text);

// bool move_through_list_of_points(std::vector<geometry_msgs::Pose>& waypoints, 
//                                  moveit::planning_interface::MoveGroupInterface& move_group,
//                                  const robot_state::JointModelGroup* joint_model_group,
//                                  moveit_visual_tools::MoveItVisualTools& visual_tools,
//                                  geometry_msgs::Pose& brain_model_pose,
//                                  double velociy_scaling = 1.0,
//                                  double eef_step = 0.001, // step to discretize in task space
//                                  std::string action = "default action",
//                                  bool show_brain = true);

// void clean_exit(moveit::planning_interface::MoveGroupInterface& move_group,
//                 moveit_visual_tools::MoveItVisualTools& visual_tools);

// void pointCallback(const ros_igtl_bridge::igtlpoint::ConstPtr& msg);
// // ************************************************************************************************************
// // ************************************************************************************************************
// ************************************************************************************************************
// ************************************************************************************************************
void set_needle_orientation_for_start_and_end_pose(geometry_msgs::Pose& initial_pose, geometry_msgs::Pose& target_pose)
  {
  // Get the direction of needle insertion
  tf2::Vector3 insertion_direction(target_pose.position.x - initial_pose.position.x,
                                   target_pose.position.y - initial_pose.position.y,
                                   target_pose.position.z - initial_pose.position.z);
  // Get the angle to rotate the end_effector
  double rotation_angle = needle_orientation_in_end_effector_frame.angle(insertion_direction);
  // Get the axix to rotate the end_effector
  tf2::Vector3 rotation_axis = needle_orientation_in_end_effector_frame.cross(insertion_direction);
  // Declare and compute the Quaternion for the orientation of the needle
  tf2::Quaternion needle_orientation;
  needle_orientation.setRotation(rotation_axis, rotation_angle);
  // Set orientation for start and end pose
  initial_pose.orientation.x = needle_orientation.x();
  initial_pose.orientation.y = needle_orientation.y();
  initial_pose.orientation.z = needle_orientation.z();
  initial_pose.orientation.w = needle_orientation.w();
  target_pose.orientation.x = needle_orientation.x();
  target_pose.orientation.y = needle_orientation.y();
  target_pose.orientation.z = needle_orientation.z();
  target_pose.orientation.w = needle_orientation.w();
}

void set_constraint_from_initial_orientation(geometry_msgs::Pose& initial_pose,
                                             moveit::planning_interface::MoveGroupInterface& move_group) {
  // Declare the constraint
  moveit_msgs::OrientationConstraint Needle_ocm;
  Needle_ocm.link_name = move_group.getEndEffectorLink();
  Needle_ocm.header.frame_id = move_group.getPlanningFrame();
  Needle_ocm.orientation = initial_pose.orientation;
  Needle_ocm.absolute_x_axis_tolerance = 0.01;
  Needle_ocm.absolute_y_axis_tolerance = 0.01;
  Needle_ocm.absolute_z_axis_tolerance = 0.01;
  Needle_ocm.weight = 1.0;
  // Set it as the path constraint for the group.
  moveit_msgs::Constraints Needle_constraints;
  Needle_constraints.orientation_constraints.push_back(Needle_ocm);
  move_group.setPathConstraints(Needle_constraints);
}

void show_friendly_GUI_text(moveit_visual_tools::MoveItVisualTools& visual_tools, std::string text) {
  // Show user friendly Text to control the UI
  visual_tools.publishText(text_pose, text, rvt::PURPLE, rvt::XXXLARGE, 3);
  visual_tools.trigger(); // Sending visualization
  visual_tools.prompt(text);

}

bool move_through_list_of_points(std::vector<geometry_msgs::Pose>& waypoints, 
                                 moveit::planning_interface::MoveGroupInterface& move_group,
                                 const robot_state::JointModelGroup* joint_model_group,
                                 moveit_visual_tools::MoveItVisualTools& visual_tools,
                                 geometry_msgs::Pose& brain_model_pose,
                                 double velociy_scaling = 1.0,
                                 double eef_step = 0.001, // step to discretize in task space
                                 std::string action = "default action",
                                 bool show_brain = true)
  {    
  move_group.setMaxVelocityScalingFactor(velociy_scaling); // change the movement speed
  // if (show_brain) {visual_tools.publishMesh(brain_model_pose, brain_mesh_file, rvt::PURPLE, slicer_scale, "mesh", 4);} // Add brain model
  show_friendly_GUI_text(visual_tools, "Preparing to " + action + "\nPress next to show the path planned");
  // Plan the trajectory;
  fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  my_plan.trajectory_ = trajectory;
  // Stop if planning fail
  if (fraction != 1.0) {
    visual_tools.publishText(text_pose, "Path planning failed!!! \nPress Next to End Planning", rvt::PURPLE, rvt::XXXLARGE, 3);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' to end the planning"); // Interactive UI
    return false;
  }
  // Visualization
  visual_tools.publishSphere(waypoints.back().position, rvt::PURPLE, rvt::XXLARGE, "Sphere", 2); // A sphere at target position
  visual_tools.publishText(text_pose, "Path to " + action + "\nPress Next to Move", rvt::PURPLE, rvt::XXXLARGE, 3);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group, rvt::TRANSLUCENT_LIGHT); // Trajectory line
  if (show_brain) {visual_tools.publishMesh(brain_model_pose, brain_mesh_file, rvt::PURPLE, slicer_scale, "mesh", 4);} // Add brain model
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' to accept the planning and " + action); // Interactive UI
  ROS_INFO_NAMED("tutorial", "%s", action.c_str()); // Interactive UI
  // Move the robot
  visual_tools.publishText(text_pose, "Executing: " + action, rvt::PURPLE, rvt::XXXLARGE, 3); // Friendly text
  visual_tools.trigger();
  move_group.execute(my_plan); // execute the plan
  show_friendly_GUI_text(visual_tools, action + ": Success!!!\nPress next to continue");
  ROS_INFO_NAMED("tutorial", "%s: Success!!!", action.c_str()); // Interactive UI
  return true;
}

void clean_exit(moveit::planning_interface::MoveGroupInterface& move_group,
                moveit_visual_tools::MoveItVisualTools& visual_tools) {
  // Clean exit
  move_group.clearPathConstraints(); // Clear Path constraint
  visual_tools.deleteAllMarkers(); // Clear Marker
  visual_tools.trigger();
  ros::shutdown(); // Shut down ros
}

void pointCallback(const ros_igtl_bridge::igtlpoint::ConstPtr& msg) {
  if (msg->name == entry_point_name) {  
    initial_pose.position.x = msg->pointdata.x*slicer_scale;
    initial_pose.position.y = msg->pointdata.y*slicer_scale;
    initial_pose.position.z = msg->pointdata.z*slicer_scale;
    ROS_INFO_NAMED("tutorial", "I got point \"%s\" from 3D slicer", msg->name.c_str());
  } else if (msg->name == target_point_name) {
    target_pose.position.x = msg->pointdata.x*slicer_scale;
    target_pose.position.y = msg->pointdata.y*slicer_scale;
    target_pose.position.z = msg->pointdata.z*slicer_scale;
    ROS_INFO_NAMED("tutorial", "I got point \"%s\" from 3D slicer", msg->name.c_str());
  }
  visual_tools_Ptr->publishArrow(initial_pose.position, target_pose.position, rvt::PURPLE, rvt::LARGE, 1);
  visual_tools_Ptr->trigger();
  return;
}

// ************************************************************************************************************
// ************************************************************************************************************

// Wokring function
shape_msgs::Mesh convert_igtlpolydata_to_shape_msgs_Mesh(const ros_igtl_bridge::igtlpolydata& data) {
  shape_msgs::Mesh output_mesh;
  int numVertices = data.points.size();
  int numTriangles = data.polygons.size();
  output_mesh.vertices.resize(numVertices);
  output_mesh.triangles.resize(numTriangles);
  for (int i = 0; i < numVertices; i++) {
      output_mesh.vertices[i].x = static_cast<double>(data.points[i].x);
      output_mesh.vertices[i].y = static_cast<double>(data.points[i].y);
      output_mesh.vertices[i].z = static_cast<double>(data.points[i].z);
  }
  for (int i = 0; i < numTriangles; i++) {
      output_mesh.triangles[i].vertex_indices[0] = static_cast<uint32_t>(data.polygons[i].x);
      output_mesh.triangles[i].vertex_indices[1] = static_cast<uint32_t>(data.polygons[i].x);
      output_mesh.triangles[i].vertex_indices[2] = static_cast<uint32_t>(data.polygons[i].x);
  }
  return output_mesh;
}


void polydataCallback(const ros_igtl_bridge::igtlpolydata::ConstPtr& msg) {
  // Ros only handles polydata in msg->points and msg->polygons
  // Raise warning if the polydata received have other geometries
  if (msg->strips.size() != 0) {
  ROS_WARN_NAMED("Polydata mismatch", "The polydata is expected to have only points and polygons data. \"strips\" or triangles data are sent from 3D slicer");
  }
  if (msg->lines.size() != 0) {
  ROS_WARN_NAMED("Polydata mismatch", "The polydata is expected to have only points and polygons data. \"lines\" or lines data are sent from 3D slicer");
  }
  if (msg->verts.size() != 0) {
  ROS_WARN_NAMED("Polydata mismatch", "The polydata is expected to have only points and polygons data. \"verts\" or vertices data are sent from 3D slicer");
  }
  ROS_INFO_NAMED("tutorial", "I got polydata message with %lu points, %lu polygons, %lu triangles, %lu lines and %lu vertices",
                  msg->points.size(),
                  msg->polygons.size(),
                  msg->strips.size(),
                  msg->lines.size(),
                  msg->verts.size());

  ROS_INFO_NAMED("tutorial", "Polygon information %.2f x, %.2f y, %.2f z",
                  msg->polygons[0].x,
                  msg->polygons[0].y,
                  msg->polygons[0].z);

  shape_msgs::Mesh brain_model_mesh = convert_igtlpolydata_to_shape_msgs_Mesh(*msg);
  brain_point_cloud.header.stamp = ros::Time::now();
  // Convert mesh to point cloud
  int numVertices = msg->points.size();
  brain_point_cloud.points.resize(numVertices);
  for (int i = 0; i < numVertices; i++) {
    brain_point_cloud.points[i].x = msg->points[i].x*slicer_scale;
    brain_point_cloud.points[i].y = msg->points[i].y*slicer_scale;
    brain_point_cloud.points[i].z = msg->points[i].z*slicer_scale;
  }
  // brain_point_cloud.points = msg->points;
  brain_point_cloud_pub.publish(brain_point_cloud);

  ROS_INFO_NAMED("tutorial", "Mesh vertices size %lu, triangles size %lu, two number %f, %u",
                  brain_model_mesh.vertices.size(),
                  brain_model_mesh.triangles.size(),
                  brain_model_mesh.vertices[0].x,
                  brain_model_mesh.triangles[100].vertex_indices[0]);

  // geometry_msgs::Pose pose_msg;
  // pose_msg.orientation.w = 1.0;
  // bool check = visual_tools_Ptr->publishCollisionMesh(pose_msg, "brain_model", brain_model_mesh, rvt::PURPLE);
  // ROS_INFO_NAMED("tutorial", "Send mesh: %d", check);

  return;
}
void pointcloudCallback(const ros_igtl_bridge::igtlpointcloud::ConstPtr& msg) {
  ROS_INFO_NAMED("tutorial", "I got point cloud message");
  return;
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
  // Msg to get information from IGT link (Get data from 3D slicer)
  ros_igtl_bridge::igtlpoint slicer_input_points;
  ros_igtl_bridge::igtlpolydata slicer_input_polydata;
  ros_igtl_bridge::igtlpointcloud slicer_input_point_cloud;
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
  // Get entry point, target point name and 3D slicer scale
  ros::param::get("/entry_point_name", entry_point_name);
  ros::param::get("/target_point_name", target_point_name);
  ros::param::get("/slicer_scale", slicer_scale);
  // Get other parameters
  ros::param::get("/planning_time_limit", planning_time_limit); // Get the planning time limit
  ros::param::get("/jump_threshold", jump_threshold); // Get jump_threshold to avoid snapping of robot
  // Set single threading
  ros::AsyncSpinner spinner(1);
  spinner.start();
  // Setup the move group
  static const std::string PLANNING_GROUP = "robot_arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  move_group_Ptr = &move_group;
  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // Visualization with the package MoveItVisualTools
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools_Ptr = &visual_tools;
  visual_tools.deleteAllMarkers();
  // Remote control allows users to step through a high level script
  visual_tools.loadRemoteControl();
  // Load the marker pub to help visualisation
  visual_tools.loadMarkerPub();
  visual_tools.loadRvizMarkers();
  // Set position of text markers
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
  // Create subscriber to get data from 3D slicer
  brain_point_cloud.header.frame_id = move_group.getPlanningFrame(); // Setup brain point cloud
  ros::Subscriber igtlpoint_sub = node_handle.subscribe("/IGTL_POINT_IN", 5, pointCallback);
  ros::Subscriber igtlpolydata_sub = node_handle.subscribe("/IGTL_POLYDATA_IN", 1, polydataCallback);
  ros::Subscriber igtlpointcloud_sub = node_handle.subscribe("/IGTL_POINTCLOUD_IN", 1, pointcloudCallback);
  brain_point_cloud_pub = node_handle.advertise<sensor_msgs::PointCloud>("/point_cloud", 1);

  // ************************************
  // Control the robot
  // ************************************

  // ************************************

  // ************************************
  // User friendly welcome Text
  show_friendly_GUI_text(visual_tools, "Welcome to Needle Insert Simulation\nPress Next to continue\n");
  visual_tools.deleteAllMarkers();
  // Set the orientation base on start and end position of needle
  // after welcome text to wait for slicer to publish
  set_needle_orientation_for_start_and_end_pose(initial_pose, target_pose);

  // ************************************
  // Move to initial pose
  waypoints.resize(0);
  waypoints.push_back(move_group.getCurrentPose().pose);
  waypoints.push_back(initial_pose);
  success = move_through_list_of_points(waypoints, move_group, joint_model_group, visual_tools, brain_model_pose, 1.0, 0.01, "Move to Start Pose", false);
  if (!success) {
    show_friendly_GUI_text(visual_tools, "Shutting down\nPress next to shut down");
    clean_exit(move_group, visual_tools);
    return 0;}

  // ************************************
  // Add Brain Model to the environment
  // User friendly Text
  show_friendly_GUI_text(visual_tools, "Preparing to Add brain model\nPress Next to Add brain model");
  // Add brain model
  visual_tools.publishText(text_pose, "Brain model Added\nPress Next to continue", rvt::PURPLE, rvt::XXXLARGE, 3);
  visual_tools.publishMesh(brain_model_pose, brain_mesh_file, rvt::PURPLE, slicer_scale, "mesh", 4); // Add brain model
  ROS_INFO_NAMED("tutorial", "Added brain model into the workspace"); // Show text in RViz of status 
  visual_tools.trigger(); // Sending visualization
  visual_tools.prompt("Press 'next' to continue");

  // ************************************
  // Set path constraints to make the needle point mantain desired orientation
  set_constraint_from_initial_orientation(initial_pose, move_group);

  // ************************************
  // Insert the needle
  waypoints.resize(0);
  waypoints.push_back(move_group.getCurrentPose().pose);
  waypoints.push_back(target_pose);
  success = move_through_list_of_points(waypoints, move_group, joint_model_group, visual_tools, brain_model_pose, 1.0, 0.001, "Insert the Needle");
  if (!success) {
    show_friendly_GUI_text(visual_tools, "Shutting down");
    clean_exit(move_group, visual_tools);
    return 0;}
  // ************************************
  // Remove the needle
  waypoints.resize(0);
  waypoints.push_back(move_group.getCurrentPose().pose);
  waypoints.push_back(initial_pose);
  success = move_through_list_of_points(waypoints, move_group, joint_model_group, visual_tools, brain_model_pose, 1.0, 0.001, "Remove the Needle");
  if (!success) {
    show_friendly_GUI_text(visual_tools, "Shutting down");
    clean_exit(move_group, visual_tools);
    return 0;}

  // ************************************
  // Ending the path planning
  show_friendly_GUI_text(visual_tools, "Finished!!!\nCongratulation!!!\nPress Next to End");
  // Clean exit
  clean_exit(move_group, visual_tools);
  return 0;
}


