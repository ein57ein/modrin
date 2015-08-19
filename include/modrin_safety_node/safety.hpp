#ifndef MODRIN_SAFETY_NODE_H_
#define MODRIN_SAFETY_NODE_H_

#include "ros/ros.h"

namespace modrin_safety_node
{
   class Safety
	{

   public:
      Safety() {

         ROS_INFO("hello. modrin_safety_node here.");
      }

      virtual ~Safety() {}
   };
};
#endif //MODRIN_SAFETY_NODE_H_
