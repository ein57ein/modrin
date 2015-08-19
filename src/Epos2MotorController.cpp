/*
 * Epos2MotorController.cpp
 *
 *  Created on: Apr 11, 2012
 *      Author: Martin Seidel
 */

#include "Epos2MotorController.h"




//public
Epos2MotorController::Epos2MotorController(int nodeNr, int reverseNode, void* newDevhandle)
{
	epos2MotorSet.node = nodeNr;

	epos2MotorSet.gearRatio = 0;

	epos2MotorSet.master = 0;

	epos2MotorSet.maxRotationPerMinute = 0;

	epos2MotorSet.reverse = reverseNode;

	if (newDevhandle != 0) {
		this->devhandle = newDevhandle;
		errorOutput("constructor - devhandle is valid", debug);
	} else {
		errorOutput("new device-handle - not valid. Please try again.", fatal);
	}

}

Epos2MotorController::Epos2MotorController(int nodeNr, int reverseNode, std::string port, unsigned long timeout, unsigned long baudrate)
{
	epos2MotorSet.node = nodeNr;

	epos2MotorSet.gearRatio = 0;

	epos2MotorSet.master = 1;

	epos2MotorSet.maxRotationPerMinute = 0;
	epos2MotorSet.reverse = reverseNode;

	//ROS_ERROR("port: %s", port);

	if ( !(initCommunication(port, baudrate, timeout)) ) {
		errorOutput("get device-handle - not valid. Please try again.", fatal);
	} else {
		errorOutput("constructor - can't get a connection", debug);
	}

}

Epos2MotorController::~Epos2MotorController()
{
	unsigned int errorCode=0;
	int bit=0;

	disableEpos2();

	if (epos2MotorSet.master == 1) {
		if (devhandle != 0) {
			bit = VCS_CloseDevice(devhandle, &errorCode);
			if (!bit) {errorOutput("Can't release the Epos2-Device", error, bit, errorCode);}
		} else {
			bit = VCS_CloseAllDevices(&errorCode);
			if (!bit) {errorOutput("can't disconnect a Epos2-Device, because no Epos2 is connected", debug, bit, errorCode);}
		}
	}

	errorOutput("Epos2MotorController closed", info);

}

//epos2-functions
int Epos2MotorController::resetDevice()
{
	int bit=0;
	unsigned int errorCode=0;

	state = getEpos2State();

	(state <= fault) ? bit = VCS_ResetDevice(devhandle, epos2MotorSet.node, &errorCode) : bit = 0;

	if (!bit) {
		errorOutput("reset device", error, bit, errorCode);
		return 0;
	} else {
		state = reset;
		return 1;
	}
}

int Epos2MotorController::clearFault()
{
	int bit=0;
	unsigned int errorCode=0;

	if (state == reset) {
		bit = VCS_ClearFault(devhandle, epos2MotorSet.node, &errorCode);
	} else {
		ROS_ERROR("EPOS2-Device was not reset. Please use resetDevice() first.");
		bit = 1; //set, to get a new state...
	}

	if (!bit) {
		errorOutput("clear errors", error, bit, errorCode);
		return 0;
	} else {
		state = getEpos2State();
		return 1;
	}
}

int Epos2MotorController::testForErrorsAndPrint()
{
	unsigned char numberOfDeviceError = 0;
	unsigned int errorCode=0, deviceErrorCode, bytesRead;
	int bit = 0, bit2 = 0, noError = 1;
	short unsigned int value=0;

	bit = VCS_GetObject(devhandle, epos2MotorSet.node, 0x1001, 0x00, &value, 1, &bytesRead, &errorCode);

	//ROS_ERROR("ErrorValue: %u", value);

	if (bit != 0) {
		if (value != 0) {
			noError = 0;
			//first shift until the wanted byte is first, then bitwiseAnd with 0000 0001 and check if it's zero or one
			if ((value >> 0) & 0x01 == 1) {errorOutput("DeviceError: Generic error", error);}
			if ((value >> 1) & 0x01 == 1) {errorOutput("DeviceError: Current error", error);}
			if ((value >> 2) & 0x01 == 1) {errorOutput("DeviceError: Voltage error", error);}
			if ((value >> 3) & 0x01 == 1) {errorOutput("DeviceError: Temperature error", error);}
			if ((value >> 4) & 0x01 == 1) {errorOutput("DeviceError: Communication error", error);}
			if ((value >> 5) & 0x01 == 1) {errorOutput("DeviceError: Device profile-specific error", error);}
			if ((value >> 7) & 0x01 == 1) {errorOutput("DeviceError: Motion error", error);}
		}
	} else {
		errorOutput("Couldn't read internal error Register (0x1001).", error, 0, errorCode);
	}

	bit = VCS_GetNbOfDeviceError (devhandle, epos2MotorSet.node, &numberOfDeviceError, &errorCode);

	//ROS_ERROR("Number of DeviceErrors: %u", numberOfDeviceError);

	if(bit != 0) {
		if (numberOfDeviceError != 0) {
			noError = 0;
			//read device error code
			for(int i = 1; i <= numberOfDeviceError; i++) {

				bit2 = VCS_GetDeviceErrorCode(devhandle, epos2MotorSet.node, i, &deviceErrorCode, &errorCode);
				if(bit2 != 0) {
					errorOutput("The following Error was detected. (ErrorBit = ErrorNumber)", error, i, deviceErrorCode);
				} else {
					errorOutput("Couldn't read the deviceError with the Number behind ErrorBit.", error, i, errorCode);
				}
			}
		}
	} else {
		errorOutput("Couldn't read the number of device errors.", error, bit, errorCode);
	}

	return noError;
}

int Epos2MotorController::enableEpos2()//set enabled
{
	int bit=0;
	unsigned int errorCode=0;

	state = getEpos2State();

	if (state <= quickstop) {
		VCS_SetEnableState(devhandle, epos2MotorSet.node, &errorCode);
		state = getEpos2State();
	}

	if (state == enabled) {
		return 1;
	} else {
		errorOutput("SetEnable", error, bit, errorCode);
		return 0;
	}

}

int Epos2MotorController::disableEpos2()//set disabled
{
	int bit=0;
	unsigned int errorCode=0;

	state = getEpos2State();

	if (state <= quickstop) {
		VCS_SetDisableState(devhandle, epos2MotorSet.node, &errorCode);
		state = getEpos2State();
	}

	if (state == disabled) {
		return 1;
	} else {
		errorOutput("SetDisable", error, bit, errorCode);
		return 0;
	}

}

int Epos2MotorController::storeSettings()
{
	int bit=0;
	unsigned int errorCode=0;

	state = getEpos2State();

	if (state == disabled) {

		bit = VCS_Store(devhandle, epos2MotorSet.node, &errorCode);
		if (bit == 0) {
			errorOutput("couldn't store Parameter", warn, bit, errorCode);
			return 0;
		} else {
			errorOutput("Parameters saved");
			return 1;
		}

	} else {
		return 0;
	}
}

int Epos2MotorController::activateProfileVelocity()
{
	int bit=0, j=0;
	char mode = 0;
	unsigned int errorCode=0, bytesWritten;
	short int value = 1;

	bit = VCS_SetObject(devhandle, epos2MotorSet.node, 0x6086, 0x00, &value, 1, &bytesWritten, &errorCode);
	if (!bit) {
		errorOutput("set MotionProfileType to sin^2-ramp", error, bit, errorCode);
	}

	do {
		bit = VCS_SetOperationMode(devhandle, epos2MotorSet.node, OMD_PROFILE_VELOCITY_MODE, &errorCode);
		if (!bit) {errorOutput("setting Mode to ProfileVelocity - not possible", error, bit, errorCode);}

		j++;

		bit = VCS_GetOperationMode(devhandle, epos2MotorSet.node, &mode, &errorCode);
		if (!bit) {
			errorOutput("unable to read OperationMode", info, bit, errorCode);
		} else {
			errorOutput("read Mode", info, mode);
		}
	} while (mode != OMD_PROFILE_VELOCITY_MODE && j < 4);

	if (mode == OMD_PROFILE_VELOCITY_MODE) {

		j=0;
		do {
			bit = VCS_ActivateProfileVelocityMode(devhandle, epos2MotorSet.node, &errorCode);
			j++;
		} while (bit == 0 && j < 4);

		if (bit == 0) {
			errorOutput("can't activate ProfilVelocityMode", error, bit, errorCode);
			return 0;
		} else {
			errorOutput("activate ProfileVelocityMode successful");
			return 1;
		}

	} else {

		errorOutput("setting Mode to ProfileVelocity (unknown error) Mode", error, mode);
		return 0;
	}

}

int Epos2MotorController::changeRotationPerMinute(double targetVelocityRPM)
{
	int bit=0;
	unsigned int errorCode=0;
	double absoluteTargetVelocityRPM = fabs(targetVelocityRPM);

	if ( epos2MotorSet.maxRotationPerMinute >= absoluteTargetVelocityRPM ) {
		if (absoluteTargetVelocityRPM < 0.2) {	//stop motor
			bit = VCS_HaltVelocityMovement(devhandle, epos2MotorSet.node, &errorCode);
			if (!bit) {
				errorOutput("stop Motor", error, bit, errorCode);
				return 0;
			} else {
				errorOutput("stop motor");
				return 1;
			}
		} else {	//start motor with gearRatio*rpm
			bit = VCS_MoveWithVelocity(devhandle, epos2MotorSet.node, (long int) (targetVelocityRPM * 1000 * epos2MotorSet.gearRatio), &errorCode);
			char velString[30];
			sprintf(velString, "set Speed to %.3f rpm", targetVelocityRPM);

			if (!bit) {
				errorOutput( velString, error, bit, errorCode);
				return 0;
			} else {
				errorOutput(velString);
				return 1;
			}
		}//end rpm==0
	} else {
		ROS_WARN("speed wasn't changed. Absolute Values greater then the maximal velocity are not allowed.");
		return 0;
	}//end maxRPM >= rpm
}

//get and set
Epos2MotorController::EposState Epos2MotorController::getEpos2State()
{
	int bit=0;
	EposState temp = unknown;
	unsigned int errorCode=0;
	unsigned short tempState=0;

	bit=VCS_GetState(devhandle, epos2MotorSet.node, &tempState, &errorCode);

	if (!bit) {
		errorOutput("read State of Node", error, 0, errorCode);
		return unknown;
	} else {

		switch (tempState) {
			case ST_ENABLED:	temp = enabled;
								errorOutput("getState: enabled", debug);
								break;
			case ST_DISABLED:	temp = disabled;
								errorOutput("getState: disabled", debug);
								break;
			case ST_QUICKSTOP:	temp = quickstop;
								errorOutput("getState: quickstop", debug);
								break;
			case ST_FAULT:		temp = fault;
								errorOutput("getState: fault", debug);
								break;
		}

		return temp;
	}
}

double Epos2MotorController::getAbsolutePosition()
{
	int bit=0;
	long int value;
	unsigned int errorCode=0;

	bit = VCS_GetPositionIs(devhandle, epos2MotorSet.node, &value, &errorCode);
	if (!bit) {errorOutput("Can't read actual positionValue", warn, bit, errorCode);}

	//rotations = ((double)(value)) / (2000 * epos2MotorSet.gearRatio);

	return ((double)(value)) * 0.0005 / epos2MotorSet.gearRatio;
}

int Epos2MotorController::setGearData(gearData *parameter)
{
	int bit=0;
	unsigned int errorCode=0, bytesWritten=0;
	unsigned long value = 1;	//gearRatio in EPOS2 will set to because only the velocity limitation work but the target velocity will be set before (motor side) instead after the gear (wheel side)

	state = getEpos2State();

	if (state == disabled) {

		bit = VCS_SetObject(devhandle, epos2MotorSet.node, 0x2230, 0x01, &value, 4, &bytesWritten, &errorCode);
		if (!bit) { errorOutput("set Gear-Numerator", error, bit, errorCode);}

		bit = VCS_SetObject(devhandle, epos2MotorSet.node, 0x2230, 0x02, &value, 2, &bytesWritten, &errorCode);
		if (!bit) { errorOutput("set Gear-Denominator", error, bit, errorCode);}

		value = abs(parameter->maxRPM);
		bit = VCS_SetObject(devhandle, epos2MotorSet.node, 0x2230, 0x03, &value, 4, &bytesWritten, &errorCode);
		if (!bit) { errorOutput("set max RPM of Gear", error, bit, errorCode);}

		epos2MotorSet.gearNumerator = parameter->numerator;
		epos2MotorSet.gearDenomiantor = parameter->denominator;
		epos2MotorSet.gearRatio = epos2MotorSet.gearNumerator / epos2MotorSet.gearDenomiantor;

		return 1;
	} else {
		return 0;
	}
}

int Epos2MotorController::getGearData(gearData *parameter)
{
	int bit=0;
	unsigned int errorCode=0, bytesRead=0;
	unsigned long value;

	//init
	parameter->denominator = epos2MotorSet.gearDenomiantor;
	parameter->numerator = epos2MotorSet.gearNumerator;
	parameter->maxRPM = 1;

	state = getEpos2State();

	if (state <= fault) {

		/*bit = VCS_GetObject(devhandle, epos2MotorSet.node, 0x2230, 0x01, &(parameter->numerator), 4, &bytesRead, &errorCode);
		if (!bit) { errorOutput("couldn't read Gear-Numerator", error, bit, errorCode);}

		bit = VCS_GetObject(devhandle, epos2MotorSet.node, 0x2230, 0x02, &(parameter->denominator), 2, &bytesRead, &errorCode);
		if (!bit) { errorOutput("couldn't read Gear-Denominator", error, bit, errorCode);}*/

		bit = VCS_GetObject(devhandle, epos2MotorSet.node, 0x2230, 0x03, &value, 4, &bytesRead, &errorCode);
		if (!bit) { errorOutput("couldn't read max RPM of Gear", error, bit, errorCode);}

		parameter->maxRPM = (double)value;

		return 1;
	} else {
		return 0;
	}
}

int Epos2MotorController::setMotorData(motorData *parameter)
{
	int bit=0;
	unsigned int errorCode=0, bytesWritten=0;
	unsigned long value = abs(parameter->maxRPM);

	state = getEpos2State();

	if (state == disabled) {

		bit = VCS_SetMotorType(devhandle, epos2MotorSet.node, MT_DC_MOTOR, &errorCode);
		if (!bit) { errorOutput("can't set motorType to brushed DC", error, bit, errorCode);}

		if (parameter->maxPeakCurrent < parameter->continuousCurrent) { parameter->maxPeakCurrent = 2 * parameter->continuousCurrent;}

		bit = VCS_SetDcMotorParameter(devhandle, epos2MotorSet.node, parameter->continuousCurrent, parameter->maxPeakCurrent, parameter->thermalTimeConstantWinding, &errorCode);
		if (bit != 0) {
			ROS_DEBUG("MotorData set %s to: NominalCurrent - %i, PeakCurrent - %i, ThermalTimeConstant - %i", nodeStr, parameter->continuousCurrent, parameter->maxPeakCurrent, parameter->thermalTimeConstantWinding);
		} else {
			errorOutput("couldn't set DC-Motor parameter", warn, bit, errorCode);
		}

		bit = VCS_SetObject(devhandle, epos2MotorSet.node, 0x6410, 0x04, &value, 4, &bytesWritten, &errorCode);
		if (bit != 0) {
			ROS_DEBUG("Maximum Velocity set %s to: %li rpm", nodeStr, parameter->maxRPM);
		} else {
			errorOutput("couldn't set maximum RPM of Motor", warn, bit, errorCode);
		}

		return 1;
	} else {
		return 0;
	}
}

int Epos2MotorController::getMotorData(motorData *parameter)
{
	int bit=0;
	unsigned int bytesRead=0;
	unsigned int errorCode=0;
	unsigned long value;

	state = getEpos2State();

	if (state <= fault) {

		bit = VCS_GetDcMotorParameter(devhandle, epos2MotorSet.node, &(parameter->continuousCurrent), &(parameter->maxPeakCurrent), &(parameter->thermalTimeConstantWinding), &errorCode);
		if (bit != 0) {
			ROS_DEBUG("MotorData %s: NominalCurrent - %i, PeakCurrent - %i, ThermalTimeConstant - %i", nodeStr, parameter->continuousCurrent, parameter->maxPeakCurrent, parameter->thermalTimeConstantWinding);
		} else {
			errorOutput("couldn't read DC-Motor parameter", warn, bit, errorCode);
		}

		bit = VCS_GetObject(devhandle, epos2MotorSet.node, 0x6410, 0x04, &value, 4, &bytesRead, &errorCode);
		if (bit != 0) {
			ROS_DEBUG("Maximum Velocity is %s: %li rpm", nodeStr, parameter->maxRPM);
		} else {
			errorOutput("couldn't read maximum RPM of Motor", warn, bit, errorCode);
		}

		parameter->maxRPM = (double) value;

		return 1;
	} else {
		return 0;
	}
}

int Epos2MotorController::setAccelerationData(accelerationData *parameter)
{
	int bit = 0, notNegativ = 0;
	unsigned int errorCode, bytesWritten;
	unsigned int value, value2;

	if (errorIfNotBiggerZero(parameter->quickstopDeceleration, "quickstopDeceleration") && errorIfNotBiggerZero(parameter->profileAcceleration, "profileAcceleration")) {
		notNegativ = 1;
	}
	if (errorIfNotBiggerZero(parameter->profileDeceleration, "profileDeceleration") && errorIfNotBiggerZero(parameter->maxAcceleration, "maxAcceleration") && notNegativ) {
		notNegativ = 1;
	} else {
		notNegativ = 0;
	}

	if (notNegativ == 1) {
		//setting-functions-units are [rev/(min * s)] and input-units are [m/(s^2)]

		value = (unsigned int) (parameter->maxAcceleration * 60 * epos2MotorSet.gearRatio / epos2MotorSet.wheelPerimeter);
		bit = VCS_SetObject(devhandle, epos2MotorSet.node, 0x60c5, 0x00, &value, 2, &bytesWritten, &errorCode);
		if (!bit) {
			errorOutput("set max Acceleration", error, bit, errorCode);
		}

		value = (unsigned int) (parameter->profileAcceleration * 60 * epos2MotorSet.gearRatio / epos2MotorSet.wheelPerimeter);
		value2 = (unsigned int) (parameter->profileDeceleration * 60 * epos2MotorSet.gearRatio / epos2MotorSet.wheelPerimeter);
		bit = VCS_SetVelocityProfile(devhandle, epos2MotorSet.node, value, value2, &errorCode);
		if (!bit) {
			errorOutput("set velocity profile acceleration and deceleration", error, bit, errorCode);
		}

		value = (unsigned int) (parameter->quickstopDeceleration * 60 * epos2MotorSet.gearRatio / epos2MotorSet.wheelPerimeter);
		bit = VCS_SetObject(devhandle, epos2MotorSet.node, 0x6085, 0x00, &value, 2, &bytesWritten, &errorCode);
		if (!bit) {
			errorOutput("set max QuickStopDeceleration", error, bit, errorCode);
		}

		return 1;
	} else {
		errorOutput("No Acceleration Values will be set because at least one parameter is not valid.");
		return 0;
	}
}

int Epos2MotorController::getAccelerationData(accelerationData *parameter)
{
	int bit = 0;
	unsigned int errorCode, bytesRead;
	unsigned int value, value2;

	//init
	parameter->maxAcceleration = 0.51;
	parameter->profileAcceleration = 0.4;
	parameter->profileDeceleration = 0.4;
	parameter->quickstopDeceleration = 0.5;

	//getting-functions-units are [rev/(min * s)] and output-units are [m/(s^2)]

	bit = VCS_GetObject(devhandle, epos2MotorSet.node, 0x60c5, 0x00, &value, 2, &bytesRead, &errorCode);
	if (!bit) {
		errorOutput("get max Acceleration", error, bit, errorCode);
	} else {
		parameter->maxAcceleration = ((double) value) * epos2MotorSet.wheelPerimeter / (60 * epos2MotorSet.gearRatio);
	}

	bit = VCS_GetVelocityProfile(devhandle, epos2MotorSet.node, &value, &value2, &errorCode);
	if (!bit) {
		errorOutput("get velocity profile acceleration and deceleration", error, bit, errorCode);
	} else {
		parameter->profileAcceleration = ((double) value) * epos2MotorSet.wheelPerimeter / (60 * epos2MotorSet.gearRatio);
		parameter->profileDeceleration = ((double) value2) * epos2MotorSet.wheelPerimeter / (60 * epos2MotorSet.gearRatio);
	}

	bit = VCS_GetObject(devhandle, epos2MotorSet.node, 0x6085, 0x00, &value, 2, &bytesRead, &errorCode);
	if (!bit) {
		errorOutput("get max QuickStopDeceleration", error, bit, errorCode);
	} else {
		parameter->quickstopDeceleration = ((double) value) * epos2MotorSet.wheelPerimeter / (60 * epos2MotorSet.gearRatio);
	}

	return 1;
}

int Epos2MotorController::setMaxRPM(double maxRPM)
{
	int bit=0;
	unsigned int errorCode=0, bytesWritten=0;
	unsigned long value = abs(maxRPM*1000*epos2MotorSet.gearRatio);

	state = getEpos2State();

	if (state == disabled) {

		bit = VCS_SetObject(devhandle, epos2MotorSet.node, 0x607f, 0x00, &value, 4, &bytesWritten, &errorCode);
		if (!bit) {
			errorOutput("set max RPM of wheels", error, bit, errorCode);
		} else {
			epos2MotorSet.maxRotationPerMinute = maxRPM;
		}

		ROS_INFO("maxRPM set to: %.4lf", maxRPM);

		return 1;
	} else {
		return 0;
	}
}

int Epos2MotorController::getMaxRPM(double *maxRPM)
{
	int bit=0;
	unsigned int errorCode=0, bytesRead=0;
	long int value;

	*maxRPM = 0;
	state = getEpos2State();

	if (state <= fault) {

		bit = VCS_GetObject(devhandle, epos2MotorSet.node, 0x607f, 0x00, &value, 4, &bytesRead, &errorCode);
		if (!bit) {
			errorOutput("couldn't read max RPM of wheels", error, bit, errorCode);
		}

		*maxRPM = (double)value * 0.001 / epos2MotorSet.gearRatio;

		return 1;
	} else {
		return 0;
	}
}

int Epos2MotorController::getRegulatorParameter(regulator *parameter)
{
	int bit=0, bit1=0;
	unsigned int errorCode=0, bytesRead=0;

	state = getEpos2State();

	if (state <= fault) {

		bit = VCS_GetPositionRegulatorGain(devhandle, epos2MotorSet.node, &(*parameter).positionGain.p, &(*parameter).positionGain.i, &(*parameter).positionGain.d, &errorCode);
		if (bit != 0) {
			ROS_DEBUG("PositionGain read %s: P - %i, I - %i, D - %i", nodeStr, (*parameter).positionGain.p, (*parameter).positionGain.i, (*parameter).positionGain.d);
		} else {
			errorOutput("couldn't read Position Gain Values", warn,  bit, errorCode);
		}

		bit = VCS_GetPositionRegulatorFeedForward(devhandle, epos2MotorSet.node, &(*parameter).positionFF.Velocity, &(*parameter).positionFF.Acceleration, &errorCode);
		if (bit != 0) {
			ROS_DEBUG("PositionFeedForward read %s: Velocity - %i, Acceleration - %i", nodeStr, (*parameter).positionFF.Velocity, (*parameter).positionFF.Acceleration);
		} else {
			errorOutput("couldn't read Position FeedForward Values", warn,  bit, errorCode);
		}

		bit = VCS_GetVelocityRegulatorGain(devhandle, epos2MotorSet.node, &(*parameter).velocityGain.p, &(*parameter).velocityGain.i, &errorCode);
		if (bit != 0) {
			ROS_DEBUG("VelocityGain read %s: P - %i, I - %i", nodeStr, (*parameter).velocityGain.p, (*parameter).velocityGain.i);
		} else {
			errorOutput("couldn't read Velocity Gain Values", warn, bit, errorCode);
		}

		bit  = VCS_GetObject(devhandle, epos2MotorSet.node, 0x60f9, 0x04, (short unsigned int *) &(*parameter).velocityFF.Velocity, 2, &bytesRead, &errorCode);
		bit1 = VCS_GetObject(devhandle, epos2MotorSet.node, 0x60f9, 0x05, (short unsigned int *) &(*parameter).velocityFF.Acceleration, 2, &bytesRead, &errorCode);
		if (bit != 0 && bit1 !=0) {
			ROS_DEBUG("VelocityFeedForward read %s: Velocity - %i, Acceleration - %i", nodeStr, (*parameter).velocityFF.Velocity, (*parameter).velocityFF.Acceleration);
		} else {
			errorOutput("couldn't read Velocity FeedForward Values", warn, bit, errorCode);
		}

		bit = VCS_GetCurrentRegulatorGain(devhandle, epos2MotorSet.node, &(*parameter).currentGain.p, &(*parameter).currentGain.i, &errorCode);
		if (bit != 0) {
			ROS_DEBUG("CurrentGain read %s: P - %i, I - %i", nodeStr, (*parameter).currentGain.p, (*parameter).currentGain.i);
		} else {
			errorOutput("couldn't read Current Gain Values", warn, bit, errorCode);
		}

		return 1;
	} else {
		return 0;
	}
}

int Epos2MotorController::setRegulatorParameter(regulator *parameter)
{
	int bit=0, bit1 = 0;
	unsigned int errorCode=0, bytesWritten=0;

	state = getEpos2State();

	if (state == disabled) {

		bit = VCS_SetPositionRegulatorGain(devhandle, epos2MotorSet.node, (*parameter).positionGain.p, (*parameter).positionGain.i, (*parameter).positionGain.d, &errorCode);
		if (bit != 0) {
			ROS_INFO("PositionGain was set %s to: P - %i, I - %i, D - %i", nodeStr, (*parameter).positionGain.p, (*parameter).positionGain.i, (*parameter).positionGain.d);
		} else {
			errorOutput("couldn't write Position Gain Values", warn, bit, errorCode);
		}

		bit = VCS_SetPositionRegulatorFeedForward(devhandle, epos2MotorSet.node, (*parameter).positionFF.Velocity, (*parameter).positionFF.Acceleration, &errorCode);
		if (bit != 0) {
			ROS_INFO("PositionFeedForward was set %s to: Velocity - %i, Acceleration - %i", nodeStr, (*parameter).positionFF.Velocity, (*parameter).positionFF.Acceleration);
		} else {
			errorOutput("couldn't write Position FeedForward Values", warn, bit, errorCode);
		}

		bit = VCS_SetVelocityRegulatorGain(devhandle, epos2MotorSet.node, (*parameter).velocityGain.p, (*parameter).velocityGain.i, &errorCode);
		if (bit != 0) {
			ROS_INFO("VelocityGain was set %s to: P - %i, I - %i", nodeStr, (*parameter).velocityGain.p, (*parameter).velocityGain.i);
		} else {
			errorOutput("couldn't write Velocity Gain Values", warn, bit, errorCode);
		}

		bit = VCS_SetObject(devhandle, epos2MotorSet.node, 0x60f9, 0x04, (short unsigned int *) &(*parameter).velocityFF.Velocity, 2, &bytesWritten, &errorCode);
		bit1 = VCS_SetObject(devhandle, epos2MotorSet.node, 0x60f9, 0x05, (short unsigned int *) &(*parameter).velocityFF.Acceleration, 2, &bytesWritten, &errorCode);
		if (bit != 0 && bit1 !=0) {
			ROS_INFO("VelocityFeedForward was set %s to: Velocity - %i, Acceleration - %i", nodeStr, (*parameter).velocityFF.Velocity, (*parameter).velocityFF.Acceleration);
		} else {
			errorOutput("couldn't write Velocity FeedForward Values", warn, bit, errorCode);
		}

		bit = VCS_SetCurrentRegulatorGain(devhandle, epos2MotorSet.node, (*parameter).currentGain.p, (*parameter).currentGain.i, &errorCode);
		if (bit != 0) {
			ROS_INFO("CurrentGain was set %s to: P - %i, I - %i", nodeStr, (*parameter).currentGain.p, (*parameter).currentGain.i);
		} else {
			errorOutput("couldn't write Current Gain Values", warn, bit, errorCode);
		}

		return 1;
	} else {
		return 0;
	}
}

//private

int Epos2MotorController::setSpin(int reverse)
{
	int bit=0;
	unsigned int errorCode=0, bytesWritten=0;
	unsigned long value;

	state = getEpos2State();

	if (state == disabled) {

		value = (reverse == 1) ? 0x0100 : 0x0000;

		bit = VCS_SetObject(devhandle, epos2MotorSet.node, 0x2008, 0x00, &value, 2, &bytesWritten, &errorCode);
		if (!bit) {
			errorOutput("change Spin", error, bit, errorCode);
			return 0;
		} else {
			return 1;
		}
	} else {
		return 0;
	}

}

int Epos2MotorController::setDimensionAndNotation()
{
	int bit=0;
	unsigned int errorCode=0, bytesWritten=0;
	short signValue = 0;
	unsigned short unsignValue = 0;

	state = getEpos2State();

	if (state == disabled) {

		unsignValue = 0xac;
		bit = VCS_SetObject(devhandle, epos2MotorSet.node, 0x608a, 0x00, &unsignValue, 1, &bytesWritten, &errorCode);
		if (!bit) { errorOutput("set Position Dimension Index", error, bit, errorCode);}

		unsignValue = 0xa4;
		bit = VCS_SetObject(devhandle, epos2MotorSet.node, 0x608c, 0x00, &unsignValue, 2, &bytesWritten, &errorCode);
		if (!bit) { errorOutput("set Velocity Dimension Index", error, bit, errorCode);}

		//unsignValue = 0xa4;
		bit = VCS_SetObject(devhandle, epos2MotorSet.node, 0x608e, 0x00, &unsignValue, 1, &bytesWritten, &errorCode);
		if (!bit) { errorOutput("set Acceleration Dimension Index", error, bit, errorCode);}

		signValue = 0x0;
		bit = VCS_SetObject(devhandle, epos2MotorSet.node, 0x6089, 0x00, &signValue, 1, &bytesWritten, &errorCode);
		if (!bit) { errorOutput("set Position Notation Index", error, bit, errorCode);}

		signValue = 0xfd;
		bit = VCS_SetObject(devhandle, epos2MotorSet.node, 0x608b, 0x00, &signValue, 1, &bytesWritten, &errorCode);
		if (!bit) { errorOutput("set Velocity Notation Index", error, bit, errorCode);}

		signValue = 0x0;
		bit = VCS_SetObject(devhandle, epos2MotorSet.node, 0x608d, 0x00, &signValue, 1, &bytesWritten, &errorCode);
		if (!bit) { errorOutput("set Acceleration Notation Index", error, bit, errorCode);}

		return 1;
	} else {
		return 0;
	}
}

void Epos2MotorController::errorOutput(std::string errorText, int errorLevel, long int errorBit, unsigned int errorCode)
{
	int bit=0;					//control-bit for the return of functions
	unsigned short maxStr=100;	//max stringsize
	char errorInfo[maxStr];		//errorstring
	char errorStr[200];

	if (errorBit != 1 || errorCode != 0) {
		bit = VCS_GetErrorInfo(errorCode, errorInfo, maxStr);	//get the ErrorString
		if (bit) {	//Output for known Errors
			sprintf(errorStr, " with Errorbit: %li & Error: %s (0x%x)", errorBit, errorInfo, errorCode);
		} else {	//Output if there is no ErrorString
			ROS_ERROR("Can't read ErrorCode with the VCS_GetErrorInfo-Function from the EPOS2-Library!");
			sprintf(errorStr, " with Errorbit: %li & Error: NO DESCRIPTION (0x%x)", errorBit, errorCode);
		}
	} else {
		bit = 1;
		sprintf(errorStr,"");
	}

	switch(errorLevel) {
	case debug: ROS_DEBUG("[%s] %s%s", nodeStr, errorText.c_str(), errorStr);
				break;
	case info:  ROS_INFO("[%s] %s%s", nodeStr, errorText.c_str(), errorStr);
				break;
	case warn:  ROS_WARN("[%s] %s%s", nodeStr, errorText.c_str(), errorStr);
				break;
	case error: ROS_ERROR("[%s] %s%s", nodeStr, errorText.c_str(), errorStr);
				break;
	case fatal: ROS_FATAL("[%s] %s%s", nodeStr, errorText.c_str(), errorStr);
				break;
	default: 	ROS_WARN("Unknown Errorlevel: [%s] %s%s", nodeStr, errorText.c_str(), errorStr);
				break;
	}
}

void Epos2MotorController::init(double maxRPM, double wheelPerimeter)
{
	if ( disableEpos2() ) {
		epos2MotorSet.wheelPerimeter = wheelPerimeter;

		setSpin(epos2MotorSet.reverse);
		setDimensionAndNotation();

		gearData gearTemp = {7000, 147, 2};
		setGearData(&gearTemp);
		motorData motorTemp = {5770, 12000, 10, 7000};
		setMotorData(&motorTemp);
		accelerationData accTemp = {2.0, 2.0, 3.5, 3.52};
		setAccelerationData(&accTemp);
		setMaxRPM(maxRPM);

		storeSettings();

		enableEpos2();
	} else {
		errorOutput("can't initialize device because it's not in disable state", error);
	}
}

int Epos2MotorController::initCommunication(std::string port, unsigned long baudrate, unsigned long timeout)
{
	int bit=0;
	unsigned int errorCode=0;

	//ROS_ERROR("port: %s, %i", port.c_str(), port.substr(0,3).compare("USB"));

	if ( port.substr(0,3).compare("USB") == 0 ) {
		if (baudrate == 0) {baudrate = 1000000;}
		if (timeout == 0) {timeout = 750;}
		devhandle = VCS_OpenDevice((char*)"EPOS2", (char*)"MAXON SERIAL V2", (char*)"USB", (char*) port.c_str(), &errorCode);
	} else if ( port.substr(0,8).compare("/dev/tty") == 0 ) {
		if (baudrate == 0) {baudrate = 115200;}
		if (timeout == 0) {timeout = 750;}
		devhandle = VCS_OpenDevice((char*)"EPOS2", (char*)"MAXON_RS232", (char*)"RS232", (char*) port.c_str(), &errorCode);
	} else {
		devhandle = 0;
	}

	this->devhandle=devhandle;

	nodeStr=(char*)"EPOS2";

	if (devhandle==0) {
		errorOutput("can't get devhandle - please check the port settings (mostly in the parameter server)", fatal, 0, 0x10000003);
		//errorOutput("open Device", fatal, (int) devhandle, errorCode);
		return 0;
	} else {
		errorOutput("open Device successfully", info, (long int) devhandle, errorCode);

		//set communication parameter
		bit = VCS_SetProtocolStackSettings(devhandle, baudrate, timeout, &errorCode);
		if (!bit) {
			errorOutput("can't set ProtocollStackSettings", fatal, bit, errorCode);
			this->devhandle = 0;
			return 0;
		} else {
			errorOutput("set ProtocollStackSettings successfully", debug);
			return 1;
		}
	}
}

int Epos2MotorController::errorIfNotBiggerZero(double number, std::string varName)
{
	if (number > 0) {
		return 1;
	} else {
		errorOutput("Parameter " + varName + " isn't greater than zero. Please change it before running the EPOS2.", error);
		return 0;
	}
}



















