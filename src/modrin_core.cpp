#include "modrin/modrin_core.hpp"

namespace modrin
{
   ModrinCore::ModrinCore(ros::NodeHandle roshandle):motor_loader("modrin", "modrin::Motor"),roshandle(roshandle),spinner(4) {

      ROS_INFO("hello. modrin_core here.");

      pthread_create(&left_t, NULL, &ModrinCore::start, this);
      pthread_create(&right_t, NULL, &ModrinCore::start, this);

      ROS_INFO("hello. modrin_core constructed.");

      //spinner.spin();
      spinner.start();
      std::cout<<"modrin_core start spinning.\n";
      pthread_join(left_t, NULL);
      pthread_join(right_t, NULL);
      std::cout<<"modrin_core threads joint.\n";
      ros::waitForShutdown();
      std::cout<<"stop spinning\n";
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

            motor_node1->initialize(roshandle, "left_epos2");//temp.str().c_str() );
         } else {
            motor_node2.reset();
            motor_node2 = motor_loader.createInstance("modrin_motor_plugins::Epos2");

            temp.str("");
            temp << ros::names::clean(ros::this_node::getName()) << "/" << "right_epos2";

            motor_node2->initialize(roshandle, "right_epos2");//temp.str().c_str() );
         }

         ROS_INFO("motor loaded");
      } catch(pluginlib::PluginlibException& ex) {
         ROS_FATAL("failed to load motor. Error: \"%s\"\nGoodbye.", ex.what());
      }


   }

}
