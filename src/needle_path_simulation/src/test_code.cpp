#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "triangle_marker_publisher");
    ros::NodeHandle nh;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Rate rate(1);  // 1 Hz

    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "triangle_list";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    // Define triangle vertices
    geometry_msgs::Point point1;
    point1.x = 0;
    point1.y = 0;
    point1.z = 1;

    geometry_msgs::Point point2;
    point2.x = 1;
    point2.y = 0;
    point2.z = 1;

    geometry_msgs::Point point3;
    point3.x = 0;
    point3.y = 1;
    point3.z = 1;

    geometry_msgs::Point point4;
    point3.x = 0;
    point3.y = 1;
    point3.z = 0.5;

    marker.points.push_back(point1);
    marker.points.push_back(point2);
    marker.points.push_back(point3);

    marker.points.push_back(point1);
    marker.points.push_back(point2);
    marker.points.push_back(point4);

    marker.points.push_back(point1);
    marker.points.push_back(point3);
    marker.points.push_back(point4);

    marker.points.push_back(point2);
    marker.points.push_back(point3);
    marker.points.push_back(point4);

   while (ros::ok()) {
        marker.header.stamp = ros::Time::now();
        marker_pub.publish(marker);
        rate.sleep();
    }

    return 0;
}
