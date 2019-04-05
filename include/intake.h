#include "api.h"

#ifndef _INTAKE_H_
#define _INTAKE_H_

#define INTAKE_PORT 14
#define INDEXER_PORT 16
#define INTAKE_SONAR_PORT_PING 'C'
#define INTAKE_SONAR_PORT_ECHO 'D'
#define INDEXER_SONAR_PORT_PING 'A'
#define INDEXER_SONAR_PORT_ECHO 'B'

extern pros::Motor intake;
extern pros::Motor indexer;
extern pros::ADIUltrasonic intakeSonar;
extern pros::ADIUltrasonic indexerSonar;

/***********************************Intake status global variables********************************/
extern bool intakeUpRequested; //boolean for state of intake request
extern bool intakeOutRequested;
extern bool prepareShotRequested;

/*************************************Intake status functions************************************/
extern void startIntake();
extern void startIntakeOut();
extern void stopIntake();

/********************************Intake status handling task***************************************/
extern void monitorIntake(void *param);
extern pros::Task intakeMonitor;

/*********************************Intake autonomous movement*********************************/
void runIntake(char dir, int ticks, bool waitForCompletion);

#endif