#include <pluginlib/class_list_macros.h>
#include <modrin_motor_plugins/epos2.hpp>

PLUGINLIB_EXPORT_CLASS(modrin_motor_plugins::Epos2, modrin::Motor)

namespace modrin_motor_plugins
{
   bool Epos2::onInit(ros::NodeHandle roshandle, std::string name)
   {
      ROS_INFO("ns: %s", name.c_str() );
      this->name = name;

      //add a while with initTimeout and rosspinonce
      if ( ros::param::has(name.append("/can_connected_with")) )
      {
         ROS_INFO("call srv");
      } else {
         establishCommmunication();
         //create srv
      }

      if (devhandle == 0)
      {
         return false;
      } else {
         //set parameter

         return true;
      }
   }

   bool Epos2::establishCommmunication()
   {
      std::string port;
      char *protocol, *port_type;
      int baudrate, timeout;

      ros::param::get(name.append("/port"), port);
      ros::param::param<int>(name.append("/timeout"), timeout, 750);

      if ( port.substr(0,3).compare("USB") == 0 ) {
         ros::param::param<int>(name.append("/baudrate"), baudrate, 1000000);
         protocol = (char*)"MAXON SERIAL V2";
         port_type = (char*)"USB";
      } else if ( port.substr(0,8).compare("/dev/tty") == 0 ) {
         ros::param::param<int>(name.append("/baudrate"), baudrate, 115200);
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
