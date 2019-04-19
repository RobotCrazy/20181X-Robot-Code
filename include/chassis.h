#include "api.h"

#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#define FRONT_LEFT_PORT 15
#define BACK_LEFT_PORT 17
#define FRONT_RIGHT_PORT 19
#define BACK_RIGHT_PORT 20
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
//extern float prevLeftSpeed;
//extern float prevRightSpeed;

/********************************Chassis Helper Function Declarations******************************/
extern void setRightDrive(int voltage);
extern void setLeftDrive(int voltage);
extern void setGlobalTargetAngle(int newAngle);
extern float getRotationalVelocity();

/****************************Chassis Autonomous Movement Functions*********************************/
extern void drive(char dir, float inches, int driveSpeed);
extern void drive(char dir, float inches);
extern void driveRampUp(char dir, float inches);
extern void driveRampUp(char dir, float inches, int speedTol, int posTol);
extern void driveShootAsync(char dir, float inches, int shootDistance);
extern void turnToTarget(float targetAngle, int maxSpeed);
//extern void climbPlatform(float maxInches);

/****************************Chassis General Functions**********************************/
extern void holdDrivePos(int targetPosL, int targetPosR);
extern void setRightDriveOP(int voltage);
extern void setLeftDriveOP(int voltage);

#endif