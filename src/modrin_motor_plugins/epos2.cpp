#include <pluginlib/class_list_macros.h>
#include <modrin_motor_plugins/epos2.hpp>

PLUGINLIB_EXPORT_CLASS(modrin_motor_plugins::Epos2, modrin::Motor)

namespace modrin_motor_plugins
{
   Epos2::Epos2():devhandle(0), lastEpos2ErrorCode(0)
   {
      ROS_DEBUG("created an instance of epos2-plugin for modrin");
   }

   Epos2::~Epos2()
   {
      if ( devhandle != 0 ) closeEpos2();
   }

   bool Epos2::onInit(ros::NodeHandle roshandle, std::string name)
   {
      this->name = name;
      ros::Duration d(1.0);

      if ( ros::param::has(name + "/node_nr") )
      {
         int temp;
         ros::param::get(name + "/node_nr", temp);
         epos_node_nr.push_back(temp);
      } else {
         ROS_ERROR("couldn't read an Epos2 node_nr from the parameter-server at \"%s\"", (name + "/node_nr").c_str() );
         return false;
      }

      eposCanClient = ros::param::has(name + "/can_connected_with");
      std::string srv_name = name;

      if ( eposCanClient )
      {
         std::string can_connected_with;
         ros::param::get(name + "/can_connected_with", can_connected_with);
         size_t x = srv_name.find_last_of("/");
         srv_name = srv_name.substr(0, x+1) + can_connected_with;
         epos2_can = roshandle.serviceClient<modrin::epos2_can>( srv_name );
      }

      initEpos2(roshandle, srv_name);

      if (devhandle == 0)
      {
         ROS_ERROR("couldn't receive an epos2 devhandle");
         return false;
      } else {
         //set parameter

         ROS_INFO("Epos2 with node_nr %i successfully started in ns \"%s\"", epos_node_nr[0], name.c_str() );
         return true;
      }

   }

   void Epos2::periopdicControllCallback(const ros::TimerEvent& event) {
      if ( getState() == not_init ) int i=0;//init
   }

   bool Epos2::initEpos2(ros::NodeHandle roshandle, std::string srv_name)
   {
      if (eposCanClient) {
         modrin::epos2_can temp;
         temp.request.node_nr = epos_node_nr[0];
         ros::service::waitForService(srv_name, 5000);   //initTimeout
         ROS_DEBUG("call srv: %s", srv_name.c_str() );
         if (epos2_can.call(temp))
         {
            devhandle = (void*) temp.response.devhandle;
            ROS_INFO("receive epos2 devhandle %#lx", (unsigned long int) devhandle);
         } else {
            devhandle = 0;
      }
      } else {
         establishCommmunication();

         epos2_can_srv = roshandle.advertiseService(name.c_str(), &Epos2::canSrv, this);
         ROS_DEBUG("create srv: %s", name.c_str());
      }
   }

   bool Epos2::establishCommmunication()
   {
      std::string port;
      char *protocol, *port_type;
      int baudrate, timeout;

      ros::param::get(name + "/port", port);
      ros::param::param<int>(name + "/timeout", timeout, 750);

      if ( port.substr(0,3).compare("USB") == 0 ) {
         ros::param::param<int>(name + "/baudrate", baudrate, 1000000);
         protocol = (char*)"MAXON SERIAL V2";
         port_type = (char*)"USB";
      } else if ( port.substr(0,8).compare("/dev/tty") == 0 ) {
         ros::param::param<int>(name + "/baudrate", baudrate, 115200);
         protocol = (char*)"MAXON_RS232";
         port_type = (char*)"RS232";
      } else {
         ROS_ERROR("\"%s\" isn't a valid portname for the Epos2. Allowed are USB* or /dev/tty*", port.c_str());
         return false;
      }

      unsigned int errorCode=0;
      devhandle = VCS_OpenDevice((char*)"EPOS2", protocol, port_type, (char*) port.c_str(), &lastEpos2ErrorCode);

      if (devhandle == 0) {
         printEpos2Error();
         return false;
      } else {
         if ( VCS_SetProtocolStackSettings(devhandle, baudrate, timeout, &lastEpos2ErrorCode) ) {
            ROS_INFO("open EPOS2-Device on port %s (baudrate: %i; timeout: %i). devhandle %#lx", port.c_str(), baudrate, timeout, (unsigned long int) devhandle);
            return true;
         } else {
            printEpos2Error();
            return false;
         }
      }
   }

   bool Epos2::canSrv(modrin::epos2_can::Request &req, modrin::epos2_can::Response &res)
   {
      ROS_INFO("Epos2 with node_nr %i call the canSrv of the Epos2 with node_nr %i", req.node_nr, epos_node_nr[0]);

      res.devhandle = (unsigned long int) devhandle;

      if (devhandle == 0)
      {
         return false;
      } else {
         epos_node_nr.push_back(req.node_nr);
         resetAndClearFaultOnAllDevices();
         return true;
      }
   }

   bool Epos2::setEnable()
   {
      state temp = getState();
      if ( temp == enabled ) return true;
      if ( temp != disabled ) return false;

      if ( VCS_SetEnableState(devhandle, epos_node_nr[0], &lastEpos2ErrorCode) ) {
         return true;
      } else {
         printEpos2Error();
         return false;
      }
   }

   bool Epos2::setDisable()
   {
      state temp = getState();
      if ( temp == disabled ) return true;
      if ( temp == not_init ) return false;

      if ( VCS_SetDisableState(devhandle, epos_node_nr[0], &lastEpos2ErrorCode) ) {
         return true;
      } else {
         printEpos2Error();
         return false;
      }
   }

   bool Epos2::setQuickStop()
   {
      if ( VCS_SetQuickStopState(devhandle, epos_node_nr[0], &lastEpos2ErrorCode) ) {
         return true;
      } else {
         printEpos2Error();
         return false;
      }
   }

   modrin::Motor::state Epos2::getState()
   {
      unsigned short tempState=0;

      if ( VCS_GetState(devhandle, epos_node_nr[0], &tempState, &lastEpos2ErrorCode) ) {
         switch (tempState) {
   			case ST_ENABLED:	return enabled;
   								break;
   			case ST_DISABLED:	return disabled;
   								break;
   			case ST_QUICKSTOP:   return disabled;
   								break;
   			case ST_FAULT:
            default:	         return fault;
   								break;
   		}
      } else {
         if ( devhandle != 0 ) {
            printEpos2Error();
            closeEpos2();
         }
         return not_init;
      }

   }

   void Epos2::closeEpos2()
   {
      setDisable();
      if ( !VCS_CloseDevice(devhandle, &lastEpos2ErrorCode) ) printEpos2Error();
      devhandle = 0;
   }

   //set parameters first (notation and dimension)
   bool Epos2::setRPM(double rpm) {return false;}
   double Epos2::getMaxRPM() { return 0.0; }

   void Epos2::resetAndClearFaultOnAllDevices()
   {
      std::vector<int>::iterator it;

      for (it = epos_node_nr.begin(); it < epos_node_nr.end(); it++) {
         if ( !VCS_SetDisableState(devhandle, *it, &lastEpos2ErrorCode) ) printEpos2Error();
      }

      for (it = epos_node_nr.begin(); it < epos_node_nr.end(); it++) {
         if ( !VCS_ResetDevice(devhandle, *it, &lastEpos2ErrorCode) ) printEpos2Error();
      }

      for (it = epos_node_nr.begin(); it < epos_node_nr.end(); it++) {
         if ( !VCS_ClearFault(devhandle, *it, &lastEpos2ErrorCode) ) printEpos2Error();
      }

      for (it = epos_node_nr.begin(); it < epos_node_nr.end(); it++) {
         if ( !VCS_SetEnableState(devhandle, *it, &lastEpos2ErrorCode) ) printEpos2Error();
      }
   }

   void Epos2::printEpos2Error()
   {
      unsigned short maxStr=255; //max stringsize
   	char errorText[maxStr];   //errorstring

      if ( VCS_GetErrorInfo(lastEpos2ErrorCode, errorText, maxStr) )
      {
         ROS_ERROR("Epos2 (Node_nr %i): %s", epos_node_nr[0], errorText);
      } else {
         ROS_FATAL("Unable to resolve an errorText for the Epos2-ErrorCode %#x", lastEpos2ErrorCode);
      }
   }

}
