#include "modrin/modrin_core.hpp"

namespace modrin
{
   ModrinCore::ModrinCore(ros::NodeHandle roshandle):motor_loader("modrin", "modrin::Motor"),roshandle(roshandle) {

      ROS_INFO("hello. modrin_core here.");

      //*spinner = ros::AsyncSpinner(4);
      //*spinner = ros::MultiThreadedSpinner(4);

      pthread_create(&left_t, NULL, &ModrinCore::start, this);
      pthread_create(&right_t, NULL, &ModrinCore::start, this);

      ROS_INFO("hello. modrin_core constructed.");
   }

   void* ModrinCore::start(void *arg)
   {
      static_cast<ModrinCore*>(arg)->test(pthread_self() == static_cast<ModrinCore*>(arg)->left_t);
   }

   void ModrinCore::test(bool left)
   {
      try
      {
         std::ostringstream temp;

         if ( left )
         {
            motor_node1.reset();
            motor_node1 = motor_loader.createInstance("modrin_motor_plugins::Epos2");


            temp.str("");
            temp << ros::names::clean(ros::this_node::getName()) << "/" << "left_epos2";

            motor_node1->initialize(roshandle, temp.str().c_str() );
         } else {
            motor_node2.reset();
            motor_node2 = motor_loader.createInstance("modrin_motor_plugins::Epos2");

            temp.str("");
            temp << ros::names::clean(ros::this_node::getName()) << "/" << "right_epos2";

            motor_node2->initialize(roshandle, temp.str().c_str() );
         }

         ROS_INFO("motor loaded");
      } catch(pluginlib::PluginlibException& ex) {
         ROS_FATAL("failed to load motor. Error: \"%s\"\nGoodbye.", ex.what());
      }

      //spinner->spin();
      //spinner->start();
      //ros::waitForShutdown();
   }

}
