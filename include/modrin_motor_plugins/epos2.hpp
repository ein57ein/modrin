#ifndef MODRIN_EPOS2_PLUGIN_H_
#define MODRIN_EPOS2_PLUGIN_H_

#include <modrin/motor.hpp>
#include <modrin_motor_plugins/epos2_definitions.h>
#include <modrin/epos2_can.h>

namespace modrin_motor_plugins
{

   class Epos2 : public modrin::Motor
	{
      std::string name;
      int epos_node_nr;
      void* devhandle;
      ros::ServiceServer epos2_can_srv;
      ros::ServiceClient epos2_can;

   public:
      Epos2():devhandle(0) {
         ROS_INFO("created an instance of epos2-plugin for modrin");
      }	/**< Intentionally left empty **/

      bool onInit(ros::NodeHandle roshandle, std::string name);

      bool setEnable() {return false;}
      bool setDisable() {return false;}
      bool setQuickStop() { return false; }
      state getState() { return modrin::Motor::fault; }

      bool setRPM(double rpm) {return false;}
      double getMaxRPM() { return 0.0; }
      double getAbsolutePosition() { return 0.0; }

      virtual ~Epos2(){}	/**< Intentionally left empty **/

   private:
      bool establishCommmunication();
      bool canSrv(modrin::epos2_can::Request &req, modrin::epos2_can::Response &res);
      void printEpos2Error(unsigned int errorCode);
   };
};
#endif //MODRIN_EPOS2_PLUGIN_H_
