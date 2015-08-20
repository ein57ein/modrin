#ifndef MODRIN_CORE_H_
#define MODRIN_CORE_H_

#include "ros/ros.h"

#include "modrin/motor.hpp"
#include <pluginlib/class_loader.h>

namespace modrin
{
   class ModrinCore
	{
      pluginlib::ClassLoader<modrin::Motor> motor_loader;
      boost::shared_ptr<modrin::Motor> motor_node1, motor_node2;

   public:
      ModrinCore(ros::NodeHandle roshandle);

      virtual ~ModrinCore() {}
   };
};
#endif //MODRIN_CORE_H_
