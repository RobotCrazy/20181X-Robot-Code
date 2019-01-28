/**
 * \file main.h
 *
 * Contains common definitions and header files used throughout your PROS
 * project.
 *
 * Copyright (c) 2017-2018, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

/**
 * If defined, some commonly used enums will have preprocessor macros which give
 * a shorter, more convenient naming pattern. If this isn't desired, simply
 * comment the following line out.
 *
 * For instance, E_CONTROLLER_MASTER has a shorter name: CONTROLLER_MASTER.
 * E_CONTROLLER_MASTER is pedantically correct within the PROS styleguide, but
 * not convienent for most student programmers.
 */
#define PROS_USE_SIMPLE_NAMES

/**
 * If defined, C++ literals will be available for use. All literals are in the
 * pros::literals namespace.
 *
 * For instance, you can do `4_mtr = 50` to set motor 4's target velocity to 50
 */
#define PROS_USE_LITERALS

/**
 * You should add more #includes here
 */
#include "api.h"
//#include "okapi/api.hpp"
//#include "pros/api_legacy.h"
#include "pros/rtos.hpp"
//#include "mathUtil.h"

extern pros::Motor frontLeft;
extern pros::Motor backLeft;
extern pros::Motor frontRight;
extern pros::Motor backRight;
extern pros::Motor intake;
extern pros::Motor indexer;
extern pros::Motor flywheel;
extern pros::Motor flipper;

extern pros::ADIGyro gyro;
extern pros::ADIUltrasonic indexerSonar;
extern pros::ADIUltrasonic intakeSonar;
extern pros::Controller master;

extern int autoMode;
extern int targetFlywheelSpeed;
extern bool maintainFlywheelSpeedRequested;
extern bool flywheelOnTarget;
extern char *parameter3;
extern bool intakeUpRequested; //boolean for state of intake request
extern bool intakeOutRequested;
extern bool prepareShotRequested;
extern char *parameter2;
extern int targetShootingTicks;
extern bool shootBallRequested;
extern int globalTargetAngle;
//extern bool maintainFlywheelSpeedRequested;

//Function Externs here/////
extern bool isBetween(float number, float rangeLower, float rangeUpper);
//extern void maintainFlywheelSpeed(void *param);

//Task Externs here////////
/*pros::Task flywheelRPMMonitor;
pros::Task intakeMonitor;*/
extern pros::Task flywheelRPMMonitor;
extern pros::Task intakeMonitor;

/**
 * If you find doing pros::Motor() to be tedious and you'd prefer just to do
 * Motor, you can use the namespace with the following commented out line.
 *
 * IMPORTANT: Only the okapi or pros namespace may be used, not both
 * concurrently! The okapi namespace will export all symbols inside the pros
 * namespace.
 */
//using namespace pros;
// using namespace pros::literals;
//using namespace okapi;

/**
 * Prototypes for the competition control tasks are redefined here to ensure
 * that they can be called from user code (i.e. calling autonomous from a
 * button press in opcontrol() for testing purposes).
 */
#ifdef __cplusplus
extern "C"
{
#endif
  void autonomous(void);
  void initialize(void);
  void disabled(void);
  void competition_initialize(void);
  void opcontrol(void);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/**
 * You can add C++-only headers here
 */
//#include <iostream>
#endif

#endif // _PROS_MAIN_H_
