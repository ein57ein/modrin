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

   bool Epos2::onInit()
   {
      if ( ros::param::has(full_namespace + "/node_nr") )
      {
         int temp;
         ros::param::get(full_namespace + "/node_nr", temp);
         epos_node_nr.push_back(temp);
      } else {
         ROS_ERROR("[%s] couldn't read an Epos2 node_nr from the parameter-server at \"%s\"", name.c_str(), (full_namespace + "/node_nr").c_str() );
         return false;
      }

      eposCanClient = ros::param::has(full_namespace + "/can_connected_with");

      if ( eposCanClient )
      {
         std::string can_connected_with;
         ros::param::get(full_namespace + "/can_connected_with", can_connected_with);
         //size_t x = srv_name.find_last_of("/");
         //srv_name = srv_name.substr(0, x+1) + can_connected_with;
         srv_name = ros::this_node::getName() + "/" + can_connected_with;
         epos2_can = roshandle.serviceClient<modrin::epos2_can>( srv_name );
      } else {
         //establishCommmunication();

         srv_name = full_namespace;
         epos2_can_srv = roshandle.advertiseService(srv_name.c_str(), &Epos2::canSrv, this);
         ROS_DEBUG("[%s] create srv: %s", name.c_str(), srv_name.c_str());
      }

      controllTimer = roshandle.createTimer(ros::Rate(0.2), &Epos2::periopdicControllCallback, this);

      return initEpos2(1000);
   }

   void Epos2::periopdicControllCallback(const ros::TimerEvent& event) {
      switch ( getState() ) {
         case not_init: initEpos2();
                        break;
         case fault:   if (!eposCanClient) resetAndClearFaultOnAllDevices();
                        break;
      }
   }

   bool Epos2::initEpos2(int timeout)
   {
      if (eposCanClient) {
         modrin::epos2_can temp;
         temp.request.node_nr = epos_node_nr[0];
         ros::service::waitForService(srv_name, timeout);
         ROS_DEBUG("call srv: %s", srv_name.c_str() );
         if (epos2_can.call(temp))
         {
            devhandle = (void*) temp.response.devhandle;
            ROS_INFO("[%s] receive an epos2 device-handle: %#lx", name.c_str(), (unsigned long int) devhandle);
         } else {
            devhandle = 0;
         }
      } else {
         establishCommmunication();
      }

      if (devhandle == 0)
      {
         if (eposCanClient) {
            ROS_WARN("[%s] couldn't receive an epos2 device-handle", name.c_str());
         } else {
            ROS_ERROR("[%s] couldn't receive an epos2 device-handle", name.c_str());
         }
         return false;
      } else {
         //set parameter
         if (eposCanClient) {setParameter();}

         ROS_INFO("[%s] Epos2 with node_nr %i successfully started in ns \"%s\"", name.c_str(), epos_node_nr[0], full_namespace.c_str() );
         return true;
      }
   }

   bool Epos2::establishCommmunication()
   {
      std::string port;
      char *protocol, *port_type;
      int baudrate, timeout;

      ros::param::get(full_namespace + "/port", port);
      ros::param::param<int>(full_namespace + "/timeout", timeout, 750);

      if ( port.substr(0,3).compare("USB") == 0 ) {
         ros::param::param<int>(full_namespace + "/baudrate", baudrate, 1000000);
         protocol = (char*)"MAXON SERIAL V2";
         port_type = (char*)"USB";
      } else if ( port.substr(0,8).compare("/dev/tty") == 0 ) {
         ros::param::param<int>(full_namespace + "/baudrate", baudrate, 115200);
         protocol = (char*)"MAXON_RS232";
         port_type = (char*)"RS232";
      } else {
         ROS_ERROR("[%s] \"%s\" isn't a valid portname for the Epos2. Allowed are USB* or /dev/tty*", name.c_str(), port.c_str());
         return false;
      }

      unsigned int errorCode=0;
      devhandle = VCS_OpenDevice((char*)"EPOS2", protocol, port_type, (char*) port.c_str(), &lastEpos2ErrorCode);

      if (devhandle == 0) {
         printEpos2Error();
         return false;
      } else {
         if ( VCS_SetProtocolStackSettings(devhandle, baudrate, timeout, &lastEpos2ErrorCode) ) {
            ROS_INFO("[%s] open EPOS2-Device on port %s (baudrate: %i; timeout: %i). device-handle: %#lx", name.c_str(), port.c_str(), baudrate, timeout, (unsigned long int) devhandle);
            return true;
         } else {
            printEpos2Error();
            return false;
         }
      }
   }

   bool Epos2::canSrv(modrin::epos2_can::Request &req, modrin::epos2_can::Response &res)
   {
      ROS_INFO("[%s] Epos2 with node_nr %i call the canSrv", name.c_str(), req.node_nr);

      res.devhandle = (unsigned long int) devhandle;

      if (devhandle == 0)
      {
         return false;
      } else {
         epos_node_nr.push_back(req.node_nr);
         resetAndClearFaultOnAllDevices();
         setParameter();
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
         ROS_ERROR("[%s] %s (errorCode: %#x)", name.c_str(), errorText, lastEpos2ErrorCode);
      } else {
         ROS_FATAL("[%s] Unable to resolve an errorText for the Epos2-ErrorCode %#x", name.c_str(), lastEpos2ErrorCode);
      }
   }

   bool Epos2::setParameter()
   {
      if ( !setDisable() ) return false;

      if ( !setDimensionAndNotation() ) return false;

      if ( !checkSpin() ) return false;

      if ( !checkMotorParameter() ) return false;

      if ( !VCS_Store(devhandle, epos_node_nr[0], &lastEpos2ErrorCode) ) {
         printEpos2Error();
      }

      if ( !setEnable() ) return false;

      return true;
   }

   bool Epos2::checkMotorParameter()
   {
      short unsigned motor_type;

      if ( !VCS_GetMotorType(devhandle, epos_node_nr[0], &motor_type, &lastEpos2ErrorCode) ) {
         printEpos2Error();
      }
      //ros::param::has(full_namespace + "/motor_type")
      if ( ros::param::get(full_namespace + "/motor_type", (int&) motor_type) ) {
         if ( !VCS_SetMotorType(devhandle, epos_node_nr[0], motor_type, &lastEpos2ErrorCode) ) {
            printEpos2Error();
         }
      }

      short unsigned nominal_current = 0, max_current = 0, thermal_time_constant = 0;
      unsigned char number_of_pole_pairs = 1;
      std::string motor_str;
      if ( motor_type == MT_DC_MOTOR ) {
         motor_str = "brushed DC motor";
         if ( !VCS_GetDcMotorParameter(devhandle, epos_node_nr[0], &nominal_current, &max_current, &thermal_time_constant, &lastEpos2ErrorCode) ) {
            printEpos2Error();
         }
      } else {
         motor_str = (motor_type == MT_EC_SINUS_COMMUTATED_MOTOR) ? "EC motor sinus commutated" : "EC motor block commutated";
         if ( !VCS_GetEcMotorParameter(devhandle, epos_node_nr[0], &nominal_current, &max_current, &thermal_time_constant, &number_of_pole_pairs, &lastEpos2ErrorCode) ) {
            printEpos2Error();
         }
      }
      thermal_time_constant = 100 * thermal_time_constant;

      bool something_changed = false, successfully_set = true;
      if ( ros::param::get(full_namespace + "/motor_nominal_current", (int&) nominal_current) ) { something_changed = true; }
      if ( ros::param::get(full_namespace + "/motor_max_output_current", (int&) max_current) ) { something_changed = true; }
      if ( ros::param::get(full_namespace + "/motor_thermal_time_constant", (int&) thermal_time_constant) ) { something_changed = true; }
      std::ostringstream pole_pair_str;
      pole_pair_str.str("");

      if ( motor_type == MT_DC_MOTOR ) {
         if (something_changed) {
            if ( !VCS_SetDcMotorParameter(devhandle, epos_node_nr[0], nominal_current, max_current, (short unsigned) (thermal_time_constant / 100.0), &lastEpos2ErrorCode) ) {
               printEpos2Error();
               successfully_set = false;
            }
         }
      } else {
         if ( ros::param::get(full_namespace + "/motor_pole_pair_number", (int&) number_of_pole_pairs) ) { something_changed = true; }
         pole_pair_str << number_of_pole_pairs << " pole pairs, ";
         if (something_changed) {
            if ( !VCS_SetEcMotorParameter(devhandle, epos_node_nr[0], nominal_current, max_current, (short unsigned) (thermal_time_constant / 100.0), number_of_pole_pairs, &lastEpos2ErrorCode) ) {
               printEpos2Error();
               successfully_set = false;
            }
         }
      }

      unsigned int bytes = 0;
      int max_rpm;
      if ( !VCS_GetObject(devhandle, epos_node_nr[0], 0x6410, 0x04, &max_rpm, 4, &bytes, &lastEpos2ErrorCode) ) {
         printEpos2Error();
      }

      if ( ros::param::get(full_namespace + "/motor_max_rpm", max_rpm) ) {
         if ( !VCS_SetObject(devhandle, epos_node_nr[0], 0x6410, 0x04, &max_rpm, 4, &bytes, &lastEpos2ErrorCode) ) {
            printEpos2Error();
         }
      }

      if (successfully_set) {
         ROS_INFO("[%s] current motor parameter: %s, %.3fA nominal current, %.3fA peak current, max %i rpm, %sthermal time constant winding = %.1fs", name.c_str(), motor_str.c_str(), nominal_current/1000.0, max_current/1000.0, max_rpm, pole_pair_str.str().c_str(), thermal_time_constant/1000.0);

         return true;
      } else {
         return false;
      }
   }

   bool Epos2::setDimensionAndNotation() {
      object_data data_temp[]={
      //{VN_STANDARD, int8, 0x6089, 0}, {0xac, uint8, 0x608a, 0}, //Position [steps]
      {VN_MILLI, int8, 0x608b, 0}, //{VD_RPM, uint8, 0x608c, 0}, //Velocity [1e-3 rev/min]
      //{VN_STANDARD, int8, 0x608d, 0}, {VD_RPM, uint8, 0x608e, 0} //Acceleration [rev/min²]
      };

      std::vector<object_data> data;

      for (int i=0; i < sizeof(data_temp) / sizeof(object_data); i ++) {
         data.push_back(data_temp[i]);
      }

      return setObject(&data);
   }

   bool Epos2::checkSpin() {
      object_data data_temp = {0, uint16, 0x2008, 0};
      std::vector<object_data> data;
      data.push_back(data_temp);

      if ( !getObject(&data) ) { return false; }

      bool temp;  //true = 1; false = 0
      if ( ros::param::get(full_namespace + "/motor_reverse", temp) ) {
         if ( temp xor (data[0].value >> 8) & 1 ) {
            data[0].value = (data[0].value & 0xfeff) + temp * 0x100;

            setObject(&data);
         }
      }

      ROS_INFO("[%s] Miscellaneous Configuration Word: %#x", name.c_str(), data[0].value );
      return true;
   }

   bool Epos2::getObject(std::vector<object_data> *data) {
      bool no_error = true;
      uint32_t bytes;

      for (std::vector<object_data>::iterator it=data->begin(); it < data->end(); it++) {
         if ( it->type == int8 ) {
            int8_t value;
            if ( !VCS_GetObject(devhandle, epos_node_nr[0], it->index, it->sub_index, &value, 1, &bytes, &lastEpos2ErrorCode) ) {
               printEpos2Error();
               no_error = false;
            } else {
               it->value = value;
            }
         } else if ( it->type == uint8 ) {
            uint8_t value;
            if ( !VCS_GetObject(devhandle, epos_node_nr[0], it->index, it->sub_index, &value, 1, &bytes, &lastEpos2ErrorCode) ) {
               printEpos2Error();
               no_error = false;
            } else {
               it->value = value;
            }
         } else if ( it->type == uint16 ) {
            uint16_t value;
            if ( !VCS_GetObject(devhandle, epos_node_nr[0], it->index, it->sub_index, &value, 2, &bytes, &lastEpos2ErrorCode) ) {
               printEpos2Error();
               no_error = false;
            } else {
               it->value = value;
            }
         }
      }

      return no_error;
   }

   bool Epos2::setObject(std::vector<object_data> *data) {
      bool no_error = true;
      uint32_t bytes;

      for (std::vector<object_data>::iterator it=data->begin(); it < data->end(); it++) {
         if ( it->type == int8 ) {
            int8_t value = it->value;
            if ( !VCS_SetObject(devhandle, epos_node_nr[0], it->index, it->sub_index, &value, 1, &bytes, &lastEpos2ErrorCode) ) {
               printEpos2Error();
               no_error = false;
            }
         } else if ( it->type == uint8 ) {
            uint8_t value = it->value;
            if ( !VCS_SetObject(devhandle, epos_node_nr[0], it->index, it->sub_index, &value, 1, &bytes, &lastEpos2ErrorCode) ) {
               printEpos2Error();
               no_error = false;
            }
         } else if ( it->type == uint16 ) {
            uint16_t value = it->value;
            if ( !VCS_SetObject(devhandle, epos_node_nr[0], it->index, it->sub_index, &value, 2, &bytes, &lastEpos2ErrorCode) ) {
               printEpos2Error();
               no_error = false;
            }
         }
      }

      return no_error;
   }

}
