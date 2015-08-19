#ifndef MODRIN_EPOS2_PLUGIN_H_
#define MODRIN_EPOS2_PLUGIN_H_

#include <modrin/motor.hpp>

namespace modrin_motor_plugins
{

   class Epos2 : public modrin::Motor
	{

   public:
      Epos2() {ROS_INFO("hello. epos2 here.");}	/**< Intentionally left empty **/
      virtual ~Epos2(){}	/**< Intentionally left empty **/
   };
};
#endif //MODRIN_EPOS2_PLUGIN_H_
