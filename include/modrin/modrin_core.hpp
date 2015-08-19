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
      boost::shared_ptr<modrin::Motor> motor_node;

   public:
      ModrinCore():motor_loader("modrin", "modrin::Motor") {

         ROS_INFO("hello. modrin_core here.");

         try
      	{
      		motor_node.reset();
      		motor_node = motor_loader.createInstance("modrin_motor_plugins::Epos2");
            ROS_INFO("motor loaded");
      	} catch(pluginlib::PluginlibException& ex) {
      		ROS_FATAL("failed to load motor. Error: \"%s\"\nGoodbye.", ex.what());
      	}
      }

      virtual ~ModrinCore() {}
   };
};
#endif //MODRIN_CORE_H_
