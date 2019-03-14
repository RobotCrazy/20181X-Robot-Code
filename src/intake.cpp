#include "intake.h"
#include "main.h"

pros::Motor intake(INTAKE_PORT);
pros::Motor indexer(INDEXER_PORT, true);
pros::ADIUltrasonic intakeSonar(INTAKE_SONAR_PORT_PING, INTAKE_SONAR_PORT_ECHO);
pros::ADIUltrasonic indexerSonar(INDEXER_SONAR_PORT_PING, INDEXER_SONAR_PORT_ECHO);

/****************************************Intake status global variables***************************/
bool intakeUpRequested = false;
bool intakeOutRequested = false;
bool prepareShotRequested = false;
bool shootBallRequested = false;
int targetShootingTicks = 0;

/********************************Intake status handling task************************************/
char *parameter2;
void monitorIntake(void *param)
{
  while (true)
  {
    if (intakeUpRequested == true)
    {
      if (!(isBetween(indexerSonar.get_value(), 50, 80)))
      {
        intake.move_velocity(150);
        indexer.move_velocity(200);
      }
      else if (!(isBetween(intakeSonar.get_value(), 30, 80)))
      {
        intake.move_velocity(150);
        indexer.move_velocity(0);
      }
      else
      {
        indexer.move_velocity(0);
        intake.move_velocity(0);
        intake.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
        indexer.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
      }
    }
    else if (prepareShotRequested == true)
    {
      if (!(isBetween(indexerSonar.get_value(), 50, 80)))
      {
        intake.move_velocity(150);
        indexer.move_velocity(200);
      }
      else
      {
        intake.move_velocity(0);
        indexer.move_velocity(0);
        indexer.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
      }
    }
    else if (intakeOutRequested == true)
    {
      intake.move_velocity(-200);
      indexer.move_velocity(0);
      indexer.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
    }
    else if (shootBallRequested == true)
    {
      intake.move_relative(targetShootingTicks, 200);
    }
    else
    {
      intake.move_velocity(0);
      indexer.move_velocity(0);
      indexer.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
    }
    pros::delay(5);
  }
}

pros::Task intakeMonitor(monitorIntake, parameter2, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Intake auto movement task");
