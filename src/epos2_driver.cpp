//============================================================================
// Name        : epos2_driver.cpp
// Author      : Martin Seidel
// Version     :
// Copyright   :
// Description :
//============================================================================

#include "TankSteering.h"
#include <bondcpp/bond.h>		//for sending alive-messages
#include <signal.h>

sig_atomic_t gotSignal=0;

void killCallback(int signal)
{
	gotSignal=signal;
}

/**
 * Structure for all values read from the ros::parmaeter-server
 */
struct parameterStruct {
	int node[2];	/**< internal NodeNumbers of the EPOS2s */
	int reverse[2];	/**< defines the positive direction of the rotation. one for each EPOS2 */

	double maxMeterPerSecond;	/**< [m/s] highest allowed velocity of the robot */
	double wheelBase;	/**< [m] distance between the middle of the last and the first wheel on one side of the robot. will be read from the parameter-server. */
	double axisLength;	/**< [m] the distance between two wheels with the same axis of rotation (on opposite sites of the robot) */
	double wheelPerimeter;	/**< [m] the perimeter of one wheel - all motor-driven wheels should have the same perimeter and radius */

	std::string port; /**< the name of the port you want to connect to (USB0, USB1, ... or /dev/ttyS0, /dev/ttyS1, ...) */
	unsigned long baudrate;	/**< [bit/s] the frequency for the communication. If it's not set or zero, it will be set to 1000000 (USB) or 115200 (RS232 - Serial) */
	unsigned long timeout;	/**< [ms] If there is no response from the EPOS2 within this time, you will get a communication error. If it's zero or not set, a value of 750 will be used. */
}parameter; /**< all values from the Parameter-server */

int readParameterServer() {

	int getParam, ok = 1;
	std::string getParamChar;
	double getDoubleParam;

	if (ros::param::get("/epos2_control/leftNodeNr", getParam)) {
		parameter.node[TankSteering::left]=std::min(abs(getParam), 255);
	} else {
		parameter.node[TankSteering::left]=1;
		ROS_ERROR("couldn't read left NodeNr - will use NodeNr 1");
	}
	if (ros::param::get("/epos2_control/rightNodeNr", getParam)) {
		parameter.node[TankSteering::right]=std::min(abs(getParam), 255);
	} else {
		parameter.node[TankSteering::right]=2;
		ROS_ERROR("couldn't read right NodeNr - will use NodeNr 2");
	}

	if (ros::param::get("/epos2_control/reverseLeft", getParam)) {
		parameter.reverse[TankSteering::left]=std::min(abs(getParam), 1);
	} else {
		parameter.reverse[TankSteering::left]=0;
		ROS_ERROR("couldn't read left reverse State - will use not reversed");
	}
	if (ros::param::get("/epos2_control/reverseRight", getParam)) {
		parameter.reverse[TankSteering::right]=std::min(abs(getParam), 1);
	} else {
		parameter.reverse[TankSteering::right]=1;
		ROS_ERROR("couldn't read right reverse State - will use reversed");
	}

	if (ros::param::get("/epos2_control/max_mps", getDoubleParam)) {
		parameter.maxMeterPerSecond = std::max(fabs(getDoubleParam), 0.01);
	} else {
		ok=0;
		ROS_FATAL("couldn't read max_mps - will not start Epos2");
	}

	if (ros::param::get("/epos2_control/axisLength", getDoubleParam)) {
		parameter.axisLength = std::max(fabs(getDoubleParam), 0.01);
	} else {
		ok=0;
		ROS_FATAL("couldn't read axisLength - will not start Epos2");
	}

	if (ros::param::get("/epos2_control/wheelPerimeter", getDoubleParam)) {
		parameter.wheelPerimeter = std::max(fabs(getDoubleParam), 0.03);
	} else {
		ok=0;
		ROS_FATAL("couldn't read wheelPerimeter - will not start Epos2");
	}

		if (ros::param::get("/epos2_control/wheelBase", getDoubleParam)) {
		parameter.wheelBase = std::max(fabs(getDoubleParam), (double) (parameter.wheelPerimeter / 3) );
	} else {
		ok=0;
		ROS_FATAL("couldn't read wheelBase - will not start Epos2");
	}

	if (ros::param::get("~port", getParamChar)) {
		parameter.port = getParamChar;
	} else {
		ok=0;
		ROS_FATAL("couldn't read port - will not start Epos2");
	}

	if (ros::param::get("~timeout", getParam)) {
		parameter.timeout = std::max(abs(getParam), 10);
	} else {
		//ok=0;
		//ROS_FATAL("couldn't read timeout - will not start Epos2");
	}

	if (ros::param::get("~baud", getParam)) {
		parameter.baudrate = std::max(abs(getParam), 100);
	} else {
		//ok=0;
		//ROS_FATAL("couldn't read baud(rate) - will not start Epos2");
	}

	return ok;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "epos2_driver");
	ros::NodeHandle roshandle;
	ros::Rate loopRate = 50;

	Epos2MotorController* epos[2];


	if (readParameterServer()) {

		ROS_ERROR("maxMPS: %.4lf, wheelBase: %.4lf, axisLength: %.4lf, wheelPerimeter: %.4lf", parameter.maxMeterPerSecond, parameter.wheelBase, parameter.axisLength, parameter.wheelPerimeter);

		//ROS_ERROR("port: %s", parameter.port);
		Epos2MotorController leftEpos(parameter.node[TankSteering::left], parameter.reverse[TankSteering::left], parameter.port, parameter.timeout, parameter.baudrate);
		leftEpos.nodeStr = (char*) "left EPOS2";

		if (leftEpos.devhandle != 0) {
			Epos2MotorController rightEpos(parameter.node[TankSteering::right], parameter.reverse[TankSteering::right], leftEpos.devhandle);
			rightEpos.nodeStr = (char*) "right EPOS2";
			epos[TankSteering::left] = &leftEpos;
			epos[TankSteering::right] = &rightEpos;

			bond::Bond driverBond("epos2_bond", "motorController");	//new local bond
			driverBond.setConnectTimeout(90);
			signal(SIGTERM, &killCallback);
			signal(SIGABRT, &killCallback);
			signal(SIGBUS, &killCallback);
			signal(SIGHUP, &killCallback);
			signal(SIGILL, &killCallback);
			signal(SIGINT, &killCallback);
			signal(SIGSEGV, &killCallback);

			TankSteering tank(roshandle, epos, parameter.wheelPerimeter, parameter.axisLength, parameter.maxMeterPerSecond);
			tank.initLaserscanner(120, 0.15, 1.0);

			//example how to read and edit regulator- and motor-data
			Epos2MotorController::regulator controlParameter[2];
			Epos2MotorController::motorData motorParameter[2];
			Epos2MotorController::gearData gearParameter[2];
			Epos2MotorController::accelerationData accelerationParameter[2];
			double maxRobotRPM[2];

			for (int i=TankSteering::left; i<=TankSteering::right; i++) {
				epos[i]->disableEpos2();

				epos[i]->getRegulatorParameter( &(controlParameter[i]) );
				//controlParameter[i].positionGain.i++;
				ROS_ERROR("[main %s] (position)-> P: %i, I: %i, D: %i",(i==TankSteering::left)?"left":"right", controlParameter[i].positionGain.p, controlParameter[i].positionGain.i, controlParameter[i].positionGain.d);
				ROS_ERROR("[main %s] (feedforward)-> posFF-velo: %i, posFF-acc: %i, velFF-vel: %i, velFF-acc: %i",(i==TankSteering::left)?"left":"right", controlParameter[i].positionFF.Velocity, controlParameter[i].positionFF.Acceleration, controlParameter[i].velocityFF.Velocity, controlParameter[i].velocityFF.Acceleration);
				ROS_ERROR("[main %s] (velocity)-> P: %i, I: %i",(i==TankSteering::left)?"left":"right", controlParameter[i].velocityGain.p, controlParameter[i].velocityGain.i);
				ROS_ERROR("[main %s] (current)-> P: %i, I: %i",(i==TankSteering::left)?"left":"right", controlParameter[i].currentGain.p, controlParameter[i].currentGain.i);
				//epos[i]->setRegulatorParameter( &(controlParameter[i]) );

				epos[i]->getMotorData( &(motorParameter[i]) );
				ROS_ERROR("[main %s] (motor) maxCurrent: %i, PeakCurrent: %i, ThermalTimeConstant: %i, maxMotorSpeed: %li",(i==TankSteering::left)?"left":"right", motorParameter[i].continuousCurrent, motorParameter[i].maxPeakCurrent, motorParameter[i].thermalTimeConstantWinding, motorParameter[i].maxRPM);
				//motorParameter[i].thermalTimeConstantWinding = 10;
				//epos[i]->setMotorData( &(motorParameter[i]) );

				epos[i]->getGearData( &(gearParameter[i]) );
				ROS_ERROR("[main %s] (Gear)-> maxRPM: %li, numerator: %li, denominator: %i",(i==TankSteering::left)?"left":"right", gearParameter[i].maxRPM, gearParameter[i].numerator, gearParameter[i].denominator);

				epos[i]->getMaxRPM( &(maxRobotRPM[i]) );
				ROS_ERROR("[main %s] maxRobotRPM: %.4lf",(i==TankSteering::left)?"left":"right", maxRobotRPM[i]);

				epos[i]->getAccelerationData( &(accelerationParameter[i]) );
				ROS_ERROR("[main %s] (acceleration) maxAcc: %.3lf m/s^2, profileAcc: %.3lf m/s^2, profileDec: %.3lf m/s^2, quickstopDec: %.3lf m/s^2",(i==TankSteering::left)?"left":"right", accelerationParameter[i].maxAcceleration, accelerationParameter[i].profileAcceleration, accelerationParameter[i].profileDeceleration, accelerationParameter[i].quickstopDeceleration);

				epos[i]->enableEpos2();
			}//end of: example how to read and edit regulator- and motor-data

			driverBond.start();

			while(!gotSignal && ros::ok())
			{
				loopRate.sleep();
				ros::spinOnce();
			}

			driverBond.breakBond();

			if (gotSignal == 0) {ROS_INFO("EPOS2-Driver finished successful\n");}
			return gotSignal;

		} else {
			ROS_FATAL("can't connect to the EPOS2 - if the EPOS2 is plugged, disconnected it and plug it in again. Retry.");
			return 1;
		}

	} else {

		ROS_FATAL("unable to read all necessary roboter parameter - EPS2-Driver will be closed\n");
		return 1;
	}

}
