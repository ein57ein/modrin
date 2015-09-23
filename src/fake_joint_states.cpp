#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

sensor_msgs::JointState states;
ros::Publisher jointStatePub;
double dt = 0.1;

void periodicCallback(const ros::TimerEvent& event) {
   *states.position.begin() = *states.position.begin() + *states.velocity.begin() * dt;
   *(states.position.begin()+1) = *(states.position.begin()+1) + *(states.velocity.begin()+1) * dt;
   states.header.stamp = ros::Time::now();
   jointStatePub.publish(states);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "fakeJointStates");
	ros::NodeHandle roshandle;

	jointStatePub = roshandle.advertise<sensor_msgs::JointState>("joint_states", 10);

   states.header.frame_id="volksbot_r6";
   states.name.push_back("wheels_left");
   states.position.push_back(0);
   states.velocity.push_back(0.1);
   states.effort.push_back(0);
   states.name.push_back("wheels_right");
   states.position.push_back(0);
   states.velocity.push_back(-0.1);
   states.effort.push_back(0);

   std::string serial_number_str;
   if ( roshandle.getParam("/epos_hardware/left_epos2/serial_number", serial_number_str) ) {
      std::stringstream ss;
      ss << std::hex << serial_number_str;
      uint64_t serial_number;
      ss >> serial_number;
      ROS_INFO_STREAM("Initializing: 0x" << std::hex << serial_number);
   }

   ros::Timer periodic = roshandle.createTimer(ros::Duration(dt), periodicCallback);

   ros::spin();

	return 0;
}
