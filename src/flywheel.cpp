#include "flywheel.h"
#include "main.h"

pros::Motor flywheel(FLY_WHEEL_PORT);
const float flywheelGearingFactor = 15;

/******************************Flywheel Status Global Variables*******************************/
bool maintainFlywheelSpeedRequested = false;
bool runFlywheelAtVoltageRequested = false;
bool flywheelAutoVelControl = false;
bool flywheelOnTarget = false;
bool flywheelShotDetected = false;

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
  //float voltage = ((0.6676889314051 * targetVelocity) - 7.03063980643);

  /*if (voltage > 127)
  {
    voltage = 127;
  }
  else if (voltage < 0)
  {
    voltage = 0;
  }
  else
  {*/

  if (targetFlywheelSpeed == 2475)
  {
    return 9000;
  }
  else
    return 6000;
  //}
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
void setPrevVals(double newVelocity)
{
  //std::cout << "new Velocity: " << newVelocity;
  //if (abs(averagePrevVelocity() - newVelocity) < 200)
  //{
  for (int i = 1; i < 20; i++)
  {
    prevVelocities[i - 1] = prevVelocities[i];
  }
  prevVelocities[19] = newVelocity;
  //}
}

double getScaledFlywheelVelocity()
{
  return (flywheel.get_actual_velocity() * flywheelGearingFactor);
}

double getNewestFlywheelVelocity()
{
  return averagePrevVelocity();
}

/**
 * This function is used to detect a major drop in rpm in the flywheel 
 * to detect when a shot has been made.
 **/
void detectRPMDrop()
{
  if (flywheelOnTarget == true)
  {
    if (getScaledFlywheelVelocity() < (targetFlywheelSpeed - 100))
    {
      flywheelShotDetected = true;
    }
    else
    {
      flywheelShotDetected = false;
    }
  }
}

bool checkFlywheelOnTarget()
{
  double flywheelVelocity = getScaledFlywheelVelocity();
  if (flywheelVelocity > 2700)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/******************************Flywheel Status Handling Task**********************************/
char *parameter3;
void maintainFlywheelSpeed(void *param)
{

  //Constants//
  double kp = 2;
  double ki = .1;
  double kd = 0;

  //PID Variables Here//
  double currentVelocity = getNewestFlywheelVelocity();
  double error = targetFlywheelSpeed - currentVelocity;
  double lastError = 0;
  double totalError = 0;
  double integralActiveZone = 400;
  double proportional = 0;
  double integral = 0;

  int onTargetCount = 0;
  bool inActiveZone = false;
  bool firstVelocityError = false;

  while (true)
  {
    if (maintainFlywheelSpeedRequested == true)
    {
      setPrevVals(getScaledFlywheelVelocity());
      currentVelocity = getNewestFlywheelVelocity(); //velocity of final velocity scaled with physical
      //gearing
      error = targetFlywheelSpeed - currentVelocity;
      proportional = error * kp;

      if (abs(error) > integralActiveZone) //Out of its active zone
      {
        inActiveZone = false;
        if (currentVelocity < targetFlywheelSpeed)
        {
          totalError = 12000 / ki;
        }
        else if (currentVelocity > targetFlywheelSpeed)
        {
          totalError = 1;
        }
      }
      else //In its active zone
      {
        if (inActiveZone == false)
        {
          inActiveZone = true;
          totalError = estimateFlywheelVoltage(targetFlywheelSpeed) / ki;
        }
        totalError += error;
        if (totalError * ki > 12000)
        {
          totalError = 12000.0 / ki;
        }
      }

      integral = totalError * ki;
      currentFlywheelVoltage = proportional + integral;

      flywheel.move_voltage(currentFlywheelVoltage);

      if (flywheelShotDetected == false)
      {
        detectRPMDrop();
      }

      if (abs(error) < 80)
      {
        onTargetCount += 1;
        if (onTargetCount >= 15)
        {
          flywheelOnTarget = true;
        }
        else
        {
          flywheelOnTarget = false;
        }
      }
      else
      {
        if (firstVelocityError == false)
        {
          firstVelocityError = true;
          onTargetCount++;
        }
        else
        {
          firstVelocityError = false;
          flywheelOnTarget = false;
          onTargetCount = 0;
        }
      }

      std::cout << "vel: " << currentVelocity << "   e: " << error << "   tE: " << totalError
                << "   p: " << proportional << "   i: " << integral << "    volt: "
                << currentFlywheelVoltage << "    targ: " << flywheelOnTarget << "   co: " << onTargetCount
                << "    drop: " << flywheelShotDetected << "   targSp: " << targetFlywheelSpeed << "\n\n";
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
    while (getScaledFlywheelVelocity() < requiredSpeed ||
           getScaledFlywheelVelocity() > requiredSpeed > 5)
    {
      pros::delay(2);
    }
  }
  else
  {
    while (getScaledFlywheelVelocity() < requiredSpeed)
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
  pros::lcd::print(4, "Speed: %d", getScaledFlywheelVelocity());
  intakeMonitor.suspend();
  indexer.move_relative(intakeTicks, 200);
  intake.move_relative(intakeTicks, 200);
  pros::delay(500);
  intakeMonitor.resume();
  if (stopFlywheelOnFinish)
  {
    stopFlywheel();
  }
}