#ifndef MODRIN_MOTOR_INTERFACE_H_
#define MODRIN_MOTOR_INTERFACE_H_

#include "ros/ros.h"

namespace modrin
{
   class Motor
	{

   protected:
      Motor() {ROS_INFO("hello. modrin motor interface here.");}

   public:
      virtual ~Motor() {}
   };
};
#endif //MODRIN_MOTOR_INTERFACE_H_
