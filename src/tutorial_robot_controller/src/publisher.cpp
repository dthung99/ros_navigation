#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"

// #include <iostream>

// #include <sstream>
// #include <string>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

float getInputFromUser(std::string variable = "variable") {
    // Get input from the user
    std::cout << "Enter the target " << variable << ": ";
    std::string user_input;
    std::getline(std::cin, user_input);
    return std::stof(user_input);
}

void messageCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    std::cout << *msg;
    // Process the received message here
    // Access message fields using msg->field_name
}

int main(int argc, char **argv)
{
  // declare some optional variable
  // std::vector<std::string> stringarray;
  std::vector<std::string> stringArray = {
    "base_to_part_1",
    "arm_1_to_arm_2",
    "arm_2_to_arm_3",
    "arm_3_to_arm_4",
    "arm_4_to_arm_5",
    "arm_5_to_arm_6",
    "arm_6_to_end_effector"
  };

  std::cout << "Node start \n";
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  // Create a publisher
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1);
  ros::Publisher publisher = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  // ros::Subscriber sub = n.subscribe("joint_states", 1, messageCallback);
  ros::Rate loop_rate(10);

  std_msgs::String msg;
  sensor_msgs::JointState target;
  target.name.insert(target.name.end(),stringArray.begin(),stringArray.end());

  int count = 0;
  while (ros::ok())
  {
    std::stringstream ss;
    ss << "hello world " << count << std::endl;

    float target_x = 0.00f,
          target_y = 0.00f,
          target_z = 0.00f,
          target_tx = 0.00f,
          target_ty = 0.00f,
          target_tz = 0.00f;
    // Get input from the user
    target_x = getInputFromUser("x");
    target_y = getInputFromUser("y");
    target_z = getInputFromUser("z");
    target_tx = getInputFromUser("tx");
    target_ty = getInputFromUser("ty");
    target_tz = getInputFromUser("tz");
    // Print something
    std::cout << target_x << "\n" << target_y << "\n" << target_z << "\n";
    // Publish
    msg.data = ss.str();
    chatter_pub.publish(msg);
    target.position.clear();
    target.position.push_back(target_x);
    target.position.push_back(target_y);
    target.position.push_back(target_z);
    target.position.push_back(target_tx);
    target.position.push_back(target_ty);
    target.position.push_back(target_tz);
    target.position.push_back(0.00f);
    publisher.publish(target);

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
