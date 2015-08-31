#ifndef MODRIN_EPOS2_PLUGIN_H_
#define MODRIN_EPOS2_PLUGIN_H_

#include <modrin/motor.hpp>
#include <modrin_motor_plugins/epos2_definitions.h>
#include <modrin/epos2_can.h>

namespace modrin_motor_plugins
{

   class Epos2 : public modrin::Motor
	{
      std::vector<int> epos_node_nr;
      void* devhandle;
      unsigned int lastEpos2ErrorCode;

      ros::ServiceServer epos2_can_srv;
      ros::ServiceClient epos2_can;
      bool eposCanClient;
      std::string srv_name;
      ros::Timer controllTimer;

   public:
      Epos2();

      bool onInit();

      bool setEnable();
      bool setDisable();
      bool setQuickStop();
      state getState();
      void closeEpos2();

      bool setRPM(double rpm);
      double getMaxRPM();
      double getAbsolutePosition() { return 0.0; }

      virtual ~Epos2();

   private:

      void periopdicControllCallback(const ros::TimerEvent& event);
      bool initEpos2(int timeout = 1000);
      bool establishCommmunication();
      bool canSrv(modrin::epos2_can::Request &req, modrin::epos2_can::Response &res);
      void resetAndClearFaultOnAllDevices();

      void printEpos2Error();

      bool setParameter();
      bool setMotorParameter();
   };
};
#endif //MODRIN_EPOS2_PLUGIN_H_
