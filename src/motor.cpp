#include "modrin/motor.hpp"

namespace modrin
{
   bool Motor::initialize(ros::NodeHandle roshandle, std::string name)
   {
      return onInit(roshandle, name);
   }
}
