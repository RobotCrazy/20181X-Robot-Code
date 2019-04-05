#include "flywheel.h"
#include "main.h"

pros::Motor flywheel(FLY_WHEEL_PORT);
const float flywheelGearingFactor = 15;

/******************************Flywheel Status Global Variables*******************************/
bool maintainFlywheelSpeedRequested = false;
bool runFlywheelAtVoltageRequested = false;
bool flywheelAutoVelControl = false;
bool flywheelOnTarget = false;

/*****************************Flywheel Velocity Control Variables*****************************/
float currentFlywheelVoltage = 0;
int targetFlywheelSpeed = 0;
int targetFlywheelVoltage = 0;

double prevVelocities[20] = {};

/******************************Flywheel Status Functions**************************************/
void setFlywheelVoltage(int voltage)
{
  targetFlywheelVoltage = voltage;
}
void startFlywheelVoltage(int voltage)
{
  maintainFlywheelSpeedRequested = false;
  flywheelAutoVelControl = false;
  runFlywheelAtVoltageRequested = true;
  setFlywheelVoltage(voltage);
}
void startFlywheel(int targetSpeed)
{
  maintainFlywheelSpeedRequested = true;
  flywheelAutoVelControl = false;
  targetFlywheelSpeed = targetSpeed;
}
void startFlywheelAutoVelControl(int targetSpeed)
{
  maintainFlywheelSpeedRequested = false;
  flywheelAutoVelControl = true;
  targetFlywheelSpeed = targetSpeed;
}
void startFlywheel(int voltage, int targetSpeed)
{
  maintainFlywheelSpeedRequested = true;
  targetFlywheelSpeed = targetSpeed;
  flywheel.move_voltage(voltage);
}

void stopFlywheel()
{
  maintainFlywheelSpeedRequested = false;
  flywheelAutoVelControl = false;
  runFlywheelAtVoltageRequested = false;
  targetFlywheelSpeed = 0;
  targetFlywheelVoltage = 0;
}

void setFlywheelTargetSpeed(int speed)
{
  targetFlywheelSpeed = speed;
}

/************************************Flywheel Velocity Control Functions*************************/
float estimateFlywheelVoltage(float targetVelocity) //Fix this to be in terms of voltage
{
  float voltage = ((0.6676889314051 * targetVelocity) - 7.03063980643);

  if (voltage > 127)
  {
    voltage = 127;
  }
  else if (voltage < 0)
  {
    voltage = 0;
  }
  else
  {
    return voltage;
  }
}

double averagePrevVelocity()
{
  double average = 0;
  for (int i = 0; i < 20; i++)
  {
    average += prevVelocities[i];
  }
  average /= 20;
  return average;
}

/**
 * This function is used to detect a major drop in rpm in the flywheel 
 * to detect when a shot has been made.
 **/
void detectRPMDrop()
{
  double initialRPMAverage = averagePrevVelocity();
}

/******************************Flywheel Status Handling Task**********************************/
char *parameter3;
void maintainFlywheelSpeed(void *param)
{

  //Constants//
  float kp = .3;
  float ki = .01;
  float kd = 0;

  //PID Variables Here//
  double currentVelocity = flywheel.get_actual_velocity() * flywheelGearingFactor;
  float lastVelocity1 = 0;
  float lastVelocity2 = 0;
  float lastVelocity3 = 0;
  float averageVelocity = 0;
  double error = targetFlywheelSpeed - currentVelocity;
  double lastError = 0;
  double totalError = 0;
  double integralActiveZone = 100;
  double proportional = 0;
  double integral = 0;

  int onTargetCount = 0;

  //Temp Variable//
  int deltaTime = 0;

  while (true)
  {
    if (maintainFlywheelSpeedRequested == true)
    {
      currentVelocity = flywheel.get_actual_velocity() * flywheelGearingFactor;
      error = targetFlywheelSpeed - currentVelocity;
      proportional = error * kp;

      if (error > integralActiveZone)
      {
        totalError += error;
        integral = totalError * ki;
      }
      else
      {
        integral = estimateFlywheelVoltage(targetFlywheelSpeed);
      }
      currentFlywheelVoltage = proportional + integral;

      flywheel.move_voltage(currentFlywheelVoltage);
    }
    else if (runFlywheelAtVoltageRequested == true)
    {
      flywheel.move_voltage(targetFlywheelVoltage);
    }
    else
    {
      flywheel.move_voltage(0);
      flywheelOnTarget = false;
    }
    pros::delay(20);
  }
}

/***************************************Flywhel Status Control Task*******************************/
pros::Task flywheelRPMMonitor(maintainFlywheelSpeed, parameter3, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Flywheel speed task");

/********************************Flywheel Autonomous Movement***********************************/
/**
 * This function shoots the ball by spinning the indexer once the flywheel velocity
 * is greater than or equal to the required speed
 * requiredSpeed - The required speed at which the flywheel must shoot
 * intakeTicks - The number of ticks that the indexer must rotate to shoot the ball
 * stopFlywheelOnFinish - A boolean representing whether to shut off the flywheel after
 * shooting
 *    Pass true to shut off flywheel after shooting
 *    Pass false to allow the flywheel to continue running
 **/
void shootWhenReady(int requiredSpeed, int intakeTicks, bool stopFlywheelOnFinish)
{
  if (flywheelAutoVelControl == true)
  {
    while (flywheel.get_actual_velocity() < requiredSpeed ||
           flywheel.get_actual_velocity() > requiredSpeed > 5)
    {
      pros::delay(2);
    }
  }
  else
  {
    while (flywheel.get_actual_velocity() < requiredSpeed)
    {
      pros::delay(2);
    }
  }
  intakeMonitor.suspend();
  indexer.move_relative(intakeTicks, 200);
  pros::delay(500);
  intakeMonitor.resume();
  if (stopFlywheelOnFinish)
  {
    stopFlywheel();
  }
}

/**
 * This function shoots the ball by spinning the indexer once the flywheel speed monitor has 
 * determined that the speed is correct.
 * intakeTicks - The number of ticks that the indexer must rotate to shoot the ball
 * stopFlywheelOnFinish - A boolean representing whether to shut off the flywheel after
 * shooting
 *    Pass true to shut off flywheel after shooting
 *    Pass false to allow the flywheel to continue running
 **/
void shootWhenReady(int intakeTicks, bool stopFlywheelOnFinish)
{
  while (flywheelOnTarget == false)
  {
    pros::delay(2);
  }
  pros::lcd::print(4, "Speed: %d", flywheel.get_actual_velocity());
  intakeMonitor.suspend();
  indexer.move_relative(intakeTicks, 200);
  pros::delay(500);
  intakeMonitor.resume();
  if (stopFlywheelOnFinish)
  {
    stopFlywheel();
  }
}