#include "api.h"

#ifndef _FLYWHEEL_H_
#define _FLYWHEEL_H_

#define FLY_WHEEL_PORT 18

extern pros::Motor flywheel;
extern const float flywheelGearingFactor;

/*************************Flywheel Status Global Variables*************************************/
extern bool maintainFlywheelSpeedRequested;
extern bool runFlywheelAtVoltageRequested;
extern bool flywheelAutoVelControl;
extern bool flywheelOnTarget;
extern bool flywheelShotDetected;

/*****************************Flywheel Velocity Control Variables*****************************/
extern float currentFlywheelVoltage;
extern int targetFlywheelSpeed;
extern int targetFlywheelVoltage;

extern double prevVelocities[20];

/*************************Flywheel Status Functions******************************************/
extern void setFlywheelVoltage(int voltage);
extern void startFlywheelVoltage(int voltage);
extern void startFlywheel(int targetSpeed);
extern void startFlywheelAutoVelControl(int targetSpeed);
extern void startFlywheel(int voltage, int targetSpeed);
extern void stopFlywheel();
extern void setFlywheelTargetSpeed(int speed);

/************************************Flywheel Velocity Control Functions*************************/
extern float estimateFlywheelVoltage(float targetVelocity);
extern double averagePrevVelocity();
extern void detectRPMDrop(int newSpeed);
extern double getScaledFlywheelVelocity();

/******************************Flywheel Status Handling Task**********************************/
extern char *parameter3;
extern void maintainFlywheelSpeed(void *param);
extern pros::Task flywheelRPMMonitor;

/********************************Flywheel Autonomous Movement***********************************/
extern void shootWhenReady(int requiredSpeed, int intakeTicks, bool stopFlywheelOnFinish);
extern void shootWhenReady(int intakeTicks, bool stopFlywheelOnFinish);

#endif