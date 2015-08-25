#include <pluginlib/class_list_macros.h>
#include <modrin_motor_plugins/epos2.hpp>

PLUGINLIB_EXPORT_CLASS(modrin_motor_plugins::Epos2, modrin::Motor)

namespace modrin_motor_plugins
{
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

      if ( ros::param::has(name + "/can_connected_with") )
      {
         std::string can_connected_with, srv_name = name;
         ros::param::get(name + "/can_connected_with", can_connected_with);
         size_t x = srv_name.find_last_of("/");
         srv_name = srv_name.substr(0, x+1) + can_connected_with;
         epos2_can = roshandle.serviceClient<modrin::epos2_can>( srv_name );

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
      devhandle = VCS_OpenDevice((char*)"EPOS2", protocol, port_type, (char*) port.c_str(), &errorCode);

      if (devhandle == 0) {
         printEpos2Error(errorCode);
         return false;
      } else {
         if ( VCS_SetProtocolStackSettings(devhandle, baudrate, timeout, &errorCode) )
         {
            ROS_INFO("open EPOS2-Device on port %s (baudrate: %i; timeout: %i). devhandle %#lx", port.c_str(), baudrate, timeout, (unsigned long int) devhandle);
            return true;
         } else {
            printEpos2Error(errorCode);
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

   void Epos2::resetAndClearFaultOnAllDevices()
   {
      std::vector<int>::iterator it;
      unsigned int errorCode=0;

      for (it = epos_node_nr.begin(); it < epos_node_nr.end(); it++) {
         if ( !VCS_SetDisableState(devhandle, *it, &errorCode) ) printEpos2Error(errorCode);
      }

      for (it = epos_node_nr.begin(); it < epos_node_nr.end(); it++) {
         if ( !VCS_ResetDevice(devhandle, *it, &errorCode) ) printEpos2Error(errorCode);
      }

      for (it = epos_node_nr.begin(); it < epos_node_nr.end(); it++) {
         if ( !VCS_ClearFault(devhandle, *it, &errorCode) ) printEpos2Error(errorCode);
      }

      for (it = epos_node_nr.begin(); it < epos_node_nr.end(); it++) {
         if ( !VCS_SetEnableState(devhandle, *it, &errorCode) ) printEpos2Error(errorCode);
      }
   }

   void Epos2::printEpos2Error(unsigned int errorCode)
   {
      unsigned short maxStr=255; //max stringsize
   	char errorText[maxStr];   //errorstring

      if ( VCS_GetErrorInfo(errorCode, errorText, maxStr) )
      {
         ROS_ERROR("Epos2: %s", errorText);
      } else {
         ROS_FATAL("Unable to resolve an errorText for the Epos2-ErrorCode %#x", errorCode);
      }
   }

}
