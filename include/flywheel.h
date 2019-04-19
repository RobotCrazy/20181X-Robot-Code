#include "api.h"

#ifndef _FLYWHEEL_H_
#define _FLYWHEEL_H_

#define FLY_WHEEL_PORT 18

extern pros::Motor flywheel;
extern const double flywheelGearingFactor;

/*************************Flywheel Status Global Variables*************************************/
extern bool maintainFlywheelSpeedRequested;
extern bool runFlywheelAtVoltageRequested;
extern bool flywheelAutoVelControl;
extern bool flywheelOnTarget;
extern bool flywheelShotDetected;

/*****************************Flywheel Velocity Control Variables*****************************/
extern float currentFlywheelVoltage;
extern double flywheelStartingVelocity;
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
extern void detectRPMDrop();
extern double getScaledFlywheelVelocity();

/******************************Flywheel Status Handling Task**********************************/
extern char *parameter3;
extern void maintainFlywheelSpeedOP(void *param);
extern void maintainFlywheelSpeedAuto(void *param);
extern pros::Task flywheelRPMMonitorOP;
extern pros::Task flywheelRPMMonitorAuto;

/********************************Flywheel Autonomous Movement***********************************/
extern void shootWhenReady(int requiredSpeed, int intakeTicks, bool stopFlywheelOnFinish);
extern void shootWhenReady(int intakeTicks, bool stopFlywheelOnFinish);
extern void shootWhenReady(int requiredSpeed, int intakeTicks, bool stopFlywheelOnFinish, bool spinBothIntakes);
extern void rapidFire(int requiredSpeed, int intakeTicks, bool stopFlywheelOnFinish);

#endif