#ifndef MODRIN_SAMPLE_MOVING_H_
#define MODRIN_SAMPLE_MOVING_H_

#include "ros/ros.h"

namespace modrin_sample_moving
{
   class SampleMoving
	{

   public:
      SampleMoving() {

         ROS_INFO("hello. modrin_sample_moving here.");
      }

      virtual ~SampleMoving() {}
   };
};
#endif //MODRIN_SAMPLE_MOVING_H_
