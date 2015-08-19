#include "ros/ros.h"
#include "modrin_safety_node/safety.hpp"

int main(int argc, char **argv) {

	ros::init(argc, argv, "modrin_safety_node");
	ros::NodeHandle roshandle;

   ROS_INFO("hello. modrin_safety_node_main here.");

   modrin_safety_node::Safety modrin_safety_object;

   ros::spin();

   return 0;
}
