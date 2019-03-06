#include "api.h"

#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#define FRONT_LEFT_PORT 4
#define BACK_LEFT_PORT 7
#define FRONT_RIGHT_PORT 6
#define BACK_RIGHT_PORT 5
#define GYRO_PORT 'E'

extern pros::Motor frontLeft;
extern pros::Motor backLeft;
extern pros::Motor frontRight;
extern pros::Motor backRight;
extern pros::ADIGyro gyro;

#define WHEEL_RADIUS 2
#define WHEEL_CIRCUMFERENCE WHEEL_RADIUS * 2 * PI

/*****************************Chassis Movement Global Variables************************************/
extern int globalTargetAngle;

/********************************Chassis Helper Function Declarations******************************/
extern void setRightDrive(int voltage);
extern void setLeftDrive(int voltage);

/****************************Chassis Autonomous Movement Functions*********************************/
extern void drive(char dir, float inches, int driveSpeed);
extern void drive(char dir, float inches);

#endif