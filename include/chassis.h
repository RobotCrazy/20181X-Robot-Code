#include "api.h"

#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#define FRONT_LEFT_PORT 4
#define BACK_LEFT_PORT 7
#define FRONT_RIGHT_PORT 6
#define BACK_RIGHT_PORT 5
#define GYRO_PORT 'E'
#define ACCELEROMETER_X_PORT 'A'

extern pros::Motor frontLeft;
extern pros::Motor backLeft;
extern pros::Motor frontRight;
extern pros::Motor backRight;
extern pros::ADIGyro gyro;
extern pros::ADIAnalogIn accelerX;

#define WHEEL_RADIUS 2
#define WHEEL_CIRCUMFERENCE WHEEL_RADIUS * 2 * PI
#define GYRO_SCALE .78

/*****************************Chassis Movement Global Variables************************************/
extern int globalTargetAngle;
extern double wheelCircumference;

/********************************Chassis Helper Function Declarations******************************/
extern void setRightDrive(int voltage);
extern void setLeftDrive(int voltage);
extern void setGlobalTargetAngle(int newAngle);

/****************************Chassis Autonomous Movement Functions*********************************/
extern void drive(char dir, float inches, int driveSpeed);
extern void drive(char dir, float inches);
extern void driveRampUp(char dir, float inches);
extern void driveShootAsync(char dir, float inches, int distance1, int distance2);
extern void turnToTarget(float targetAngle, int maxSpeed);
extern void climbPlatform();

/****************************Chassis General Functions**********************************/
extern void holdDrivePos(int targetPosL, int targetPosR);

#endif