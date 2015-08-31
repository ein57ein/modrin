#include "modrin/motor.hpp"

namespace modrin
{
   bool Motor::initialize(ros::NodeHandle roshandle, std::string name)
   {
      this->name = name;
      full_namespace = ros::this_node::getName() + "/" + name;

      return onInit();
   }
}
