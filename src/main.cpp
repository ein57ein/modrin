#include "ros/ros.h"
#include "modrin/modrin_core.hpp"

int main(int argc, char **argv) {

	ros::init(argc, argv, "modrin");
	ros::NodeHandle roshandle;

   ROS_INFO("hello. modrin here.");

   modrin::ModrinCore modrin_core_object(roshandle);

   ros::spin();

   return 0;
}
