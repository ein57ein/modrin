#ifndef MODRIN_MOTOR_INTERFACE_H_
#define MODRIN_MOTOR_INTERFACE_H_

#include "ros/ros.h"

namespace modrin
{
   class Motor
	{
   protected:
      std::string name; /** name of the plugin part. should match the tf_frame_name **/
      std::string full_namespace;   /**< the namespace of the node (ros::this_node::getName()) + tf_frame_name **/
      ros::NodeHandle roshandle;

      Motor() {ROS_DEBUG("created an instance of the modrin motor interface.");}

   public:
      enum state {not_init, enabled, disabled, fault, missing_parameter};

      bool initialize(ros::NodeHandle roshandle, std::string name);
      virtual bool onInit() {}

      virtual bool setEnable() = 0;
      virtual bool setDisable() = 0;
      virtual bool setQuickStop() { setRPM(0.0); }
      virtual state getState() = 0;

      virtual bool setRPM(double rpm) = 0;
      virtual double getMaxRPM() = 0;
      virtual double getAbsolutePosition() = 0;

      virtual ~Motor() {}
   };
};
#endif //MODRIN_MOTOR_INTERFACE_H_
