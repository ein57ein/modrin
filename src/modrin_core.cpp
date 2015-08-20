#include "modrin/modrin_core.hpp"

namespace modrin
{
   ModrinCore::ModrinCore(ros::NodeHandle roshandle):motor_loader("modrin", "modrin::Motor") {

      ROS_INFO("hello. modrin_core here.");

      try
      {
         motor_node1.reset();
         motor_node1 = motor_loader.createInstance("modrin_motor_plugins::Epos2");

         std::ostringstream temp;
			temp.str("");
			temp << ros::names::clean(ros::this_node::getName()) << "/" << "left_epos2";

         motor_node1->initialize(roshandle, temp.str().c_str() );


         motor_node2.reset();
         motor_node2 = motor_loader.createInstance("modrin_motor_plugins::Epos2");

			temp.str("");
			temp << ros::names::clean(ros::this_node::getName()) << "/" << "right_epos2";

         motor_node2->initialize(roshandle, temp.str().c_str() );

         ROS_INFO("motor loaded");
      } catch(pluginlib::PluginlibException& ex) {
         ROS_FATAL("failed to load motor. Error: \"%s\"\nGoodbye.", ex.what());
      }
   }
}
