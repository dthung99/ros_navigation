#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

float getInputFromUser(std::string variable = "variable") {
    // Get input from the user
    std::cout << "Enter the target " << variable << ": ";
    std::string user_input;
    std::getline(std::cin, user_input);
    return std::stof(user_input);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);

    const double degree = M_PI/180;

    // robot state
    double tilt = 0, tinc = degree, swivel=0, angle=0, height=0, hinc=0.005;
    float joint_1 = 0.00f,
          joint_2 = 0.00f,
          joint_3 = 0.00f,
          joint_4 = 0.00f,
          joint_5 = 0.00f,
          joint_6 = 0.00f;

        // // Get input from the user
        // joint_1 = getInputFromUser("joint_1");
        // joint_2 = getInputFromUser("joint_2");
        // joint_3 = getInputFromUser("joint_3");
        // joint_4 = getInputFromUser("joint_4");
        // joint_5 = getInputFromUser("joint_5");
        // joint_6 = getInputFromUser("joint_6");

    std::vector<std::string> robotJointName = {
        "base_to_part_1",
        "arm_1_to_arm_2",
        "arm_2_to_arm_3",
        "arm_3_to_arm_4",
        "arm_4_to_arm_5",
        "arm_5_to_arm_6",
        "arm_6_to_end_effector"
    };

    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    // Declare odom message
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x = 0.0;
    odom_trans.transform.translation.y = 0.0;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0.0);
    // Declare robot message
    joint_state.name.insert(joint_state.name.end(),robotJointName.begin(),robotJointName.end());

    while (ros::ok()) {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        // update transform
        odom_trans.header.stamp = ros::Time::now();

        // Create new robot state
        // Update the message
        std::cout << joint_1 << "\n" << joint_2 << "\n" << joint_3 << "\n";
        joint_state.position.clear();
        joint_state.position.push_back(joint_1);
        joint_state.position.push_back(joint_2);
        joint_state.position.push_back(joint_3);
        joint_state.position.push_back(joint_4);
        joint_state.position.push_back(joint_5);
        joint_state.position.push_back(joint_6);
        joint_state.position.push_back(0.00f);

        //send the joint state and transform
        joint_pub.publish(joint_state);
        broadcaster.sendTransform(odom_trans);

        joint_1 += 0.1;
        joint_2 += 0.1;
        joint_3 += 0.1;
        joint_4 += 0.1;
        joint_5 += 0.1;
        joint_6 += 0.1;

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }
    return 0;
}