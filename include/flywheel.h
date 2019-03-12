#include "api.h"

#ifndef _FLYWHEEL_H_
#define _FLYWHEEL_H_

#define FLY_WHEEL 14

extern pros::Motor flywheel;

/*************************Flywheel Status Global Variables*************************************/
extern int targetFlywheelSpeed;
extern int targetFlywheelVoltage;
extern bool maintainFlywheelSpeedRequested;
extern bool runFlywheelAtVoltageRequested;
extern bool flywheelAutoVelControl;
extern bool flywheelOnTarget;

/*************************Flywheel Status Functions******************************************/
extern void setFlywheelVoltage(int voltage);
extern void startFlywheelVoltage(int voltage);
extern void startFlywheel(int targetSpeed);
extern void startFlywheelAutoVelControl(int targetSpeed);


#endif