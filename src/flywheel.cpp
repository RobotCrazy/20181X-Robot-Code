#include "flywheel.h"
#include "main.h"

pros::Motor flywheel(FLY_WHEEL_PORT);
const double flywheelGearingFactor = 15.0;

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

/***************************Flywheel Velocity Measurement**************************************/
/*double prevTime = 0;
double prevPos = 0;

double getCurrentVelocity()
{
  double currentTime = pros::millis() * 1.0;
  double currentPos = flywheel.get_position();
  double deltaTime = (currentTime - prevTime);      //Difference in time in minutes
  double deltaPos = (currentPos - prevPos) / 360.0; //in rotations
  std::cout << flywheel.get_actual_velocity() * flywheelGearingFactor << "   " << ((deltaPos / deltaTime) * 60000 * flywheelGearingFactor) << "\n\n";
  prevTime = currentTime;
  prevPos = currentPos;

  return ((deltaPos / deltaTime) * 60000 * flywheelGearingFactor);

  // if (deltaTime > 0)
  // {
  //   return (1000.0 / deltaTime) * (deltaPos * 60.0) / 900.0;
  // }
  // else
  // {
  //   return 0;
  // }
}*/

/******************************Flywheel Status Functions**************************************/
void setFlywheelVoltage(int voltage)
{
  targetFlywheelVoltage = voltage;
}
void startFlywheelVoltage(int voltage)
{
  stopFlywheel();
  runFlywheelAtVoltageRequested = true;
  setFlywheelVoltage(voltage);
}
void startFlywheel(int targetSpeed)
{
  stopFlywheel();
  maintainFlywheelSpeedRequested = true;
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
float estimateFlywheelVoltage(float targetVelocity)
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

  return (targetVelocity * 3.76);

  //}
}

double getScaledFlywheelVelocity()
{
  return (flywheel.get_actual_velocity() * flywheelGearingFactor);
}

double emaFilter(double newInput, double prevInput, double alpha)
{
  return (newInput * alpha + (1.0 - alpha) * prevInput);
}

/**
 * This function is used to detect a major drop in rpm in the flywheel 
 * to detect when a shot has been made.
 **/
void detectRPMDrop()
{
  if ((flywheelStartingVelocity - getScaledFlywheelVelocity()) > 100)
  {
    flywheelShotDetected = true;
    startFlywheelVoltage(1000);
  }
}

/******************************Flywheel Status Handling Task**********************************/
char *parameter3;
void maintainFlywheelSpeedOP(void *param)
{

  //Constants//
  double kp = .4;
  double ki = .008;
  double kd = .07;

  //PID Variables Here//
  double currentVelocity = 0; /*emaFilter(getScaledFlywheelVelocity(), lastVelocity1, 0.2)*/
  double lastVelocity1 = 0;
  double lastVelocity2 = 0;
  double lastVelocity3 = 0;
  double error = targetFlywheelSpeed - currentVelocity;
  double lastError = 0;
  double totalError = 0;
  double integralActiveZone = 300;
  double proportional = 0;
  double integral = 0;
  double derivative = 0;
  double estimate = 0;

  int onTargetCount = 0;
  bool inActiveZone = false;
  bool firstVelocityError = false;

  while (true)
  {
    if (maintainFlywheelSpeedRequested == true)
    {
      currentVelocity = emaFilter(getScaledFlywheelVelocity(), lastVelocity1, 0.2); //velocity of final velocity scaled with physical gearing
      //lv_chart_set_next(homeChart, seriesA, currentVelocity);
      error = targetFlywheelSpeed - currentVelocity;
      proportional = error * kp;
      derivative = (error - lastError) * kd;
      estimate = estimateFlywheelVoltage(targetFlywheelSpeed);

      if (abs(error) > integralActiveZone) //Out of its active zone
      {
        totalError = 0;
        if (currentVelocity < targetFlywheelSpeed)
        {
          currentFlywheelVoltage = estimateFlywheelVoltage(targetFlywheelSpeed) + 4000;
        }
        else if (currentVelocity > targetFlywheelSpeed)
        {
          currentFlywheelVoltage = estimateFlywheelVoltage(targetFlywheelSpeed) - 4000;
        }
      }
      else //In its active zone
      {
        totalError += error;
        integral = totalError * ki;
        currentFlywheelVoltage = (estimate + proportional + integral + derivative);
      }

      //currentFlywheelVoltage += proportional; // + integral;

      flywheel.move_voltage(currentFlywheelVoltage);

      /*if (flywheelShotDetected == false)
      {
        detectRPMDrop();
      }*/

      if (abs(error) < 30)
      {
        onTargetCount += 1;
        if (onTargetCount >= 30)
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
        flywheelOnTarget = false;
        onTargetCount = 0;
      }
      std::cout << currentVelocity << "\t" << error << "\t" << integral
                << "\t" << currentFlywheelVoltage << "\n";
      //std::cout << integral << "\n";
      // std::cout << "vel: " << currentVelocity << "   e: " << error << "   tE: " << totalError
      //           << "   p: " << proportional << "   i: " << integral << "    volt: "
      //           << currentFlywheelVoltage << "    targ: " << flywheelOnTarget << "   co: " << onTargetCount
      //           << "    drop: " << flywheelShotDetected << "   targSp: " << targetFlywheelSpeed << "\n\n";

      /*lastVelocity3 = lastVelocity2;
      lastVelocity2 = lastVelocity1;*/
      lastVelocity1 = currentVelocity;
      lastError = error;
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
    pros::delay(50);
  }
}

/***************************************Flywhel Status Control Task*******************************/
//pros::Task flywheelRPMMonitorOP(maintainFlywheelSpeedOP, parameter3, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Flywheel speed task");

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

  while (getScaledFlywheelVelocity() < requiredSpeed)
  {
    pros::delay(2);
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

void shootWhenBallReady(int requiredSpeed, int intakeTicks, bool stopFlywheelOnFinish)
{

  while (getScaledFlywheelVelocity() < requiredSpeed)
  {
    pros::delay(2);
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

void shootWhenReady(int requiredSpeed, int intakeTicks, bool stopFlywheelOnFinish, bool spinBothIntakes)
{

  while (getScaledFlywheelVelocity() < requiredSpeed)
  {
    pros::delay(2);
  }
  intakeMonitor.suspend();
  indexer.move_relative(intakeTicks, 200);
  if (spinBothIntakes == true)
  {
    intake.move_relative(intakeTicks, 200);
  }
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
  pros::delay(500);
  intakeMonitor.resume();
  if (stopFlywheelOnFinish)
  {
    stopFlywheel();
  }
}

void rapidFire(int requiredSpeed, int intakeTicks, bool stopFlywheelOnFinish)
{

  while (getScaledFlywheelVelocity() < requiredSpeed)
  {
    pros::delay(2);
  }
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

/***************************Flywheel Auton Speed Control**************************/
void maintainFlywheelSpeedAuto(void *param)
{

  //Constants//
  double kp = .4;
  double ki = .008;
  double kd = .07;

  //PID Variables Here//
  double currentVelocity = 0; /*emaFilter(getScaledFlywheelVelocity(), lastVelocity1, 0.2)*/
  double lastVelocity1 = 0;
  double lastVelocity2 = 0;
  double lastVelocity3 = 0;
  double error = targetFlywheelSpeed - currentVelocity;
  double lastError = 0;
  double totalError = 0;
  double integralActiveZone = 300;
  double proportional = 0;
  double integral = 0;
  double derivative = 0;
  double estimate = 0;

  int onTargetCount = 0;
  bool inActiveZone = false;
  bool firstVelocityError = false;

  while (true)
  {
    if (maintainFlywheelSpeedRequested == true)
    {
      currentVelocity = emaFilter(getScaledFlywheelVelocity(), lastVelocity1, 0.2); //velocity of final velocity scaled with physical gearing
      //lv_chart_set_next(homeChart, seriesA, currentVelocity);
      error = targetFlywheelSpeed - currentVelocity;
      proportional = error * kp;
      derivative = (error - lastError) * kd;
      estimate = estimateFlywheelVoltage(targetFlywheelSpeed);

      if (abs(error) > integralActiveZone) //Out of its active zone
      {
        totalError = 0;
        if (currentVelocity < targetFlywheelSpeed)
        {
          currentFlywheelVoltage = estimateFlywheelVoltage(targetFlywheelSpeed) + 4000;
        }
        else if (currentVelocity > targetFlywheelSpeed)
        {
          currentFlywheelVoltage = estimateFlywheelVoltage(targetFlywheelSpeed) - 4000;
        }
      }
      else //In its active zone
      {
        totalError += error;
        integral = totalError * ki;
        currentFlywheelVoltage = (estimate + proportional + integral + derivative);
      }

      //currentFlywheelVoltage += proportional; // + integral;

      flywheel.move_voltage(currentFlywheelVoltage);

      /*if (flywheelShotDetected == false)
      {
        detectRPMDrop();
      }*/

      if (abs(error) < 30)
      {
        onTargetCount += 1;
        if (onTargetCount >= 30)
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
        flywheelOnTarget = false;
        onTargetCount = 0;
      }
      std::cout << currentVelocity << "\t" << error << "\t" << integral
                << "\t" << currentFlywheelVoltage << "\n";
      //std::cout << integral << "\n";
      // std::cout << "vel: " << currentVelocity << "   e: " << error << "   tE: " << totalError
      //           << "   p: " << proportional << "   i: " << integral << "    volt: "
      //           << currentFlywheelVoltage << "    targ: " << flywheelOnTarget << "   co: " << onTargetCount
      //           << "    drop: " << flywheelShotDetected << "   targSp: " << targetFlywheelSpeed << "\n\n";

      /*lastVelocity3 = lastVelocity2;
      lastVelocity2 = lastVelocity1;*/
      lastVelocity1 = currentVelocity;
      lastError = error;
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
    pros::delay(50);
  }
}

/***************************************Flywhel Status Control Task*******************************/
pros::Task flywheelRPMMonitorAuto(maintainFlywheelSpeedAuto, parameter3, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Flywheel speed task");
