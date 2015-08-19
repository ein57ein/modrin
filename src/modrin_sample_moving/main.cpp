#include "ros/ros.h"
#include "modrin_sample_moving/sample_moving.hpp"

int main(int argc, char **argv) {

	ros::init(argc, argv, "modrin_sample_moving");
	ros::NodeHandle roshandle;

   ROS_INFO("hello. modrin_sample_moving_main here.");

   modrin_sample_moving::SampleMoving modrin_sample_moving_object;

   ros::spin();

   return 0;
}
