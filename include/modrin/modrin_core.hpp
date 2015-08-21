#ifndef MODRIN_CORE_H_
#define MODRIN_CORE_H_

#include "ros/ros.h"

#include "modrin/motor.hpp"
#include <pluginlib/class_loader.h>

#include <pthread.h>

namespace modrin
{
   class ModrinCore
	{
      pluginlib::ClassLoader<modrin::Motor> motor_loader;
      boost::shared_ptr<modrin::Motor> motor_node1, motor_node2;
      ros::NodeHandle roshandle;

       pthread_t left_t, right_t;
       //ros::AsyncSpinner *spinner;
       //ros::MultiThreadedSpinner *spinner;

   public:
      ModrinCore(ros::NodeHandle roshandle);



      virtual ~ModrinCore() {
         pthread_join(left_t, NULL);
         pthread_join(right_t, NULL);
         std::cout<<"modrin_core threads joint.";
      }

   private:
            static void* start(void *arg);
            void test(bool left);
   };
};
#endif //MODRIN_CORE_H_
