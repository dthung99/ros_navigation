// The code was adapted from https://github.com/moveit/moveit_tutorials/blob/melodic-devel/doc/move_group_interface/src/move_group_interface_tutorial.cpp
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  // ************************************
  // Set up stuff
  // ************************************
  // ************************************
  // Declare some parameters
  // Some moveit parameters
  double planning_time_limit = 10.0; // Time limit for path planning
  std::vector<geometry_msgs::Pose> waypoints; // List of via point in task space
  moveit_msgs::RobotTrajectory trajectory; // Trajectory
  const double jump_threshold = 0.0; // No large robot jump in joint space > 0.01 radian
  const double eef_step = 0.001; // Cartesian path interpolation at a resolution of 1 mm
  double fraction; // How well the end effector follow the waypoints
  bool success; // Variable to check if planning is success
  // Initial and target pose
  geometry_msgs::Pose initial_pose;
  initial_pose.orientation.w = 1.0;
  initial_pose.orientation.x = 1.0;
  initial_pose.orientation.y = 1.0;
  initial_pose.orientation.z = 1.0;
  initial_pose.position.x = 0.0;
  initial_pose.position.y = 0.5;
  initial_pose.position.z = 1.0;
  geometry_msgs::Pose target_pose;
  target_pose.orientation.w = 1.0;
  target_pose.orientation.x = 1.0;
  target_pose.orientation.y = 1.0;
  target_pose.orientation.z = 1.0;
  target_pose.position.x = 0.0;
  target_pose.position.y = 1.0;
  target_pose.position.z = 1.0;
  geometry_msgs::Pose current_pose;
  // ************************************
  // Initiate ROS
  ros::init(argc, argv, "move_group_interface_control_node");
  ros::NodeHandle node_handle;
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
  // Call the planner to compute the plan and visualize it.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  // Visualization with the package MoveItVisualTools
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();
  // Remote control allows users to step through a high level script
  visual_tools.loadRemoteControl();
  // via buttons and keyboard shortcuts in RViz
  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  Eigen::Isometry3d position = Eigen::Isometry3d::Identity(); // position in 3D for visualization
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
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start"); // Interactive UI
  // Plan the trajectory;
  waypoints.resize(0);
  waypoints.push_back(move_group.getCurrentPose().pose);
  waypoints.push_back(initial_pose);
  fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  my_plan.trajectory_ = trajectory;
  // Visualization
  visual_tools.deleteAllMarkers();
  position.translation().x() = initial_pose.position.x;
  position.translation().y() = initial_pose.position.y;
  position.translation().z() = initial_pose.position.z;
  visual_tools.publishSphere(position.translation(), rvt::PURPLE, rvt::XXLARGE);
  visual_tools.publishText(text_pose, "Path to initial pose", rvt::PURPLE, rvt::XXXXLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group, rvt::YELLOW);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to accept the planning and move robot to initial position"); // Interactive UI
  ROS_INFO_NAMED("tutorial", "Moving to initial position"); // Interactive UI
  move_group.setMaxVelocityScalingFactor(1.0); // change the speed
  move_group.execute(my_plan); // Move the robot
  ROS_INFO_NAMED("tutorial", "Moving to initial position: Success!!!"); // Interactive UI

  // ************************************
  // Move to target pose
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to plan the needle insertion path"); // Interactive UI
  move_group.setPoseTarget(target_pose); // Set target for path planning
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS); // Check if planning is success
  ROS_INFO_NAMED("tutorial", "Visualizing needle insertion path %s", success ? "" : "FAILED");
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(target_pose, "Target pose");
  visual_tools.publishText(text_pose, "Insertion path", rvt::WHITE, rvt::XXXXLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to accept the path and insert the needle"); // Interactive UI
  ROS_INFO_NAMED("tutorial", "Inserting the needle"); // Interactive UI
  move_group.setMaxVelocityScalingFactor(0.1); // change the speed
  move_group.move(); // Move the robot
  ROS_INFO_NAMED("tutorial", "Inserting the needle: Success!!!"); // Interactive UI

  // ************************************
  // Move back to initial pose
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to plan the needle remove path"); // Interactive UI
  move_group.setPoseTarget(initial_pose); // Set target for path planning
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS); // Check if planning is success
  ROS_INFO_NAMED("tutorial", "Visualizing needle remove path %s", success ? "" : "FAILED");
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(initial_pose, "Initial pose");
  visual_tools.publishText(text_pose, "Removal path", rvt::WHITE, rvt::XXXXLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to accept the path and remove the needle"); // Interactive UI
  ROS_INFO_NAMED("tutorial", "Removing the needle"); // Interactive UI
  move_group.setMaxVelocityScalingFactor(0.3); // change the speed
  move_group.move(); // Move the robot
  ROS_INFO_NAMED("tutorial", "Removing the needle: Success!!!"); // Interactive UI






  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.orientation.x = 1.0;
  target_pose1.orientation.y = 1.0;
  target_pose1.orientation.z = 1.0;
  target_pose1.position.x = 0.0;
  target_pose1.position.y = 1.0;
  target_pose1.position.z = 1.0;
  move_group.setPoseTarget(target_pose1);
  // Call the planner to compute the plan and visualize it.
  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  // Visualizing plans
  // We can also visualize the plan as a line with markers in RViz.
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  // Moving to a pose goal
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to accept the planning and move robot to initial position");
  ROS_INFO_NAMED("tutorial", "Moving to initial position");
  // move_group.move();
  ROS_INFO_NAMED("tutorial", "Moving to initial position: Success!!!");
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to go to next step");


  // ************************************
  // Planning to a joint-space goal
  // Create an pointer that references the current robot's state.
  // RobotState is the object that contains all the current position/velocity/acceleration data.
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  joint_group_positions[0] = -1.0;  // radians
  move_group.setJointValueTarget(joint_group_positions);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  // Moving to a pose goal
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to accept the planning and move robot to initial position");
  ROS_INFO_NAMED("tutorial", "Moving to initial position");
  // move_group.move();
  ROS_INFO_NAMED("tutorial", "Moving to initial position: Success!!!");
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to go to next step");

  // ************************************
  // Planning with Path Constraints
  // Path constraints can easily be specified for a link on the robot.
  // Let's specify a path constraint and a pose goal for our group.
  // First define the path constraint.
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "end_effector";
  ocm.header.frame_id = "base_link";
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;
  // Now, set it as the path constraint for the group.
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(test_constraints);

  // We will reuse the old goal that we had and plan to it.
  // Note that this will only work if the current state already
  // satisfies the path constraints. So, we need to set the start
  // state to a new pose.
  robot_state::RobotState start_state(*move_group.getCurrentState());
  geometry_msgs::Pose start_pose2;
  start_pose2.orientation.w = 1.0;
  start_pose2.position.x = 0.55;
  start_pose2.position.y = -0.05;
  start_pose2.position.z = 0.8;
  start_state.setFromIK(joint_model_group, start_pose2);
  move_group.setStartState(start_state);

  // Now we will plan to the earlier pose target from the new
  // start state that we have just created.
  move_group.setPoseTarget(target_pose1);

  // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
  // Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
  move_group.setPlanningTime(10.0);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(start_pose2, "start");
  visual_tools.publishAxisLabeled(target_pose1, "goal");
  visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");

  // When done with the path constraint be sure to clear it.
  move_group.clearPathConstraints();

  // Moving to a pose goal
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to accept the planning and move robot to initial position");
  ROS_INFO_NAMED("tutorial", "Moving to initial position");
  // move_group.move();
  ROS_INFO_NAMED("tutorial", "Moving to initial position: Success!!!");
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to go to next step");


  // Cartesian Paths
  // You can plan a Cartesian path directly by specifying a list of waypoints
  // for the end-effector to go through. Note that we are starting
  // from the new start state above.  The initial pose (start state) does not
  // need to be added to the waypoint list but adding it can help with visualizations
  // std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(start_pose2);
  geometry_msgs::Pose target_pose3 = start_pose2;
  target_pose3.position.z -= 0.2;
  waypoints.push_back(target_pose3);  // down
  target_pose3.position.y -= 0.2;
  waypoints.push_back(target_pose3);  // right
  target_pose3.position.z += 0.2;
  target_pose3.position.y += 0.2;
  target_pose3.position.x -= 0.2;
  waypoints.push_back(target_pose3);  // up and left
  // Reduce the speed of the robot arm via a scaling factor
  move_group.setMaxVelocityScalingFactor(0.1);

  // We want the Cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in Cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
  // Warning - disabling the jump threshold while operating real hardware can cause
  // large unpredictable motions of redundant joints and could be a safety issue
  // moveit_msgs::RobotTrajectory trajectory;
  // const double jump_threshold = 0.01;
  // const double eef_step = 0.01;
  fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();


  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  // Adding/Removing Objects and Attaching/Detaching Objects
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();
  // The id of the object is used to identify it.
  collision_object.id = "box1";
  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.4;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.0;
  box_pose.position.y = 1.0;
  box_pose.position.z = 1.0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;
  // collision_object.pose.position.x = 0.0;
  // collision_object.pose.position.y = 1.0;
  // collision_object.pose.position.z = 1.0;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // Now, let's add the collision object into the world
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Wait for MoveGroup to receive and process the collision object message
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

  // Now when we plan a trajectory it will avoid the obstacle
  move_group.setStartState(*move_group.getCurrentState());
  geometry_msgs::Pose another_pose;
  another_pose.orientation.w = 1.0;
  another_pose.position.x = 0.4;
  another_pose.position.y = -0.4;
  another_pose.position.z = 0.9;
  move_group.setPoseTarget(another_pose);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (pose goal move around cuboid) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");

  // Now, let's attach the collision object to the robot.
  ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
  move_group.attachObject(collision_object.id);

  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  /* Wait for MoveGroup to receive and process the attached collision object message */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object attaches to the "
                      "robot");

  // Now, let's detach the collision object from the robot.
  ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
  move_group.detachObject(collision_object.id);

  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Object dettached from robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  /* Wait for MoveGroup to receive and process the attached collision object message */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object detaches to the "
                      "robot");

  // Now, let's remove the collision object from the world.
  ROS_INFO_NAMED("tutorial", "Remove the object from the world");
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Object removed", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  /* Wait for MoveGroup to receive and process the attached collision object message */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disappears");

  // END_TUTORIAL

  ros::shutdown();
  return 0;
}