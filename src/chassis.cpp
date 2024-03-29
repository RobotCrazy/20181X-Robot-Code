#include "chassis.h"
#include "main.h"
#include "mathUtil.h"

pros::Motor frontLeft(FRONT_LEFT_PORT);
pros::Motor backLeft(BACK_LEFT_PORT);
pros::Motor frontRight(FRONT_RIGHT_PORT, true);
pros::Motor backRight(BACK_RIGHT_PORT, true);
pros::ADIGyro gyro(GYRO_PORT);

/*****************************Chassis Movement Global Variables************************************/
int globalTargetAngle = 0;
double wheelCircumference = (WHEEL_RADIUS * 2 * PI);

/*****************************Chassis Helper Functions**************************************/
void setRightDrive(int voltage)
{
  if (voltage == 0)
  {
    frontRight.move_voltage(0);
    backRight.move_voltage(0);
    frontRight.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD);
    backRight.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD);
  }
  else
  {
    frontRight.move_voltage(voltage);
    backRight.move_voltage(voltage);
  }
}

void setLeftDrive(int voltage)
{
  if (voltage == 0)
  {
    frontLeft.move_voltage(0);
    backLeft.move_voltage(0);
    frontLeft.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD);
    backLeft.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD);
  }
  else
  {
    frontLeft.move_voltage(voltage);
    backLeft.move_voltage(voltage);
  }
}

/**
 * This function accepts an int representing the real target angle.  The target is set by converting 
 * the real angle to the value the gyro will read for that angle.  
 */
void setGlobalTargetAngle(int newAngle)
{
  globalTargetAngle = (newAngle * GYRO_SCALE * 10);
}

/**********************************Chassis Autonomous Movement Functions**************************/
void drive(char dir, float inches, int driveSpeed)
{

  frontRight.tare_position();
  backRight.tare_position();
  frontLeft.tare_position();
  backLeft.tare_position();

  int ticks = (int)((inches / (PI * WHEEL_RADIUS)) * 180);

  //P Variables Here//
  int error = ticks - ((frontRight.get_position() + backRight.get_position() + frontLeft.get_position() + backLeft.get_position()) / 4);
  float lastDriveSpeed = 0;
  int angleError = 0;

  //Constants here//
  float increaseFactor = .4;

  //Tolerance Variables Here//
  int speedTolerance = 10;
  int positionTolerance = 30;

  //Deadbands//
  int speedDeadband = 2500;

  if (dir == 'b')
  {
    driveSpeed *= -1;
  }

  while (abs(frontRight.get_position() + backRight.get_position() + frontLeft.get_position() +
             backLeft.get_position()) /
             4 <
         abs(ticks))
  {
    error = ticks - ((frontRight.get_position() + backRight.get_position() + frontLeft.get_position() + backLeft.get_position()) / 4);

    if (driveSpeed > 0 && lastDriveSpeed >= 0 && driveSpeed > lastDriveSpeed)
    {
      setLeftDrive(lastDriveSpeed + increaseFactor);
      setRightDrive(lastDriveSpeed + increaseFactor);
      lastDriveSpeed += increaseFactor;
    }
    else if (driveSpeed < 0 && lastDriveSpeed <= 0 && driveSpeed < lastDriveSpeed)
    {
      setLeftDrive(lastDriveSpeed - increaseFactor);
      setRightDrive(lastDriveSpeed - increaseFactor);
      lastDriveSpeed -= increaseFactor;
    }
    else
    {
      setLeftDrive(driveSpeed);
      setRightDrive(driveSpeed);
      lastDriveSpeed = driveSpeed;
    }
  }
  setRightDrive(0);
  setLeftDrive(0);
}

/**
 * Use this function to drive for a certain number of inches
 * minSpeed - The minimum value the drive base can go at
 * Note: Set minSpeed to 0 to leave the deadband at its default
 **/
void drive(char dir, float inches)
{

  frontRight.tare_position();
  backRight.tare_position();
  frontLeft.tare_position();
  backLeft.tare_position();

  int ticks = (int)((inches / (PI * WHEEL_RADIUS)) * 180);
  int angleCorrectionFactor = 40;
  int startingAngle = globalTargetAngle;

  //P Variables Here//
  int error = ticks - ((frontRight.get_position() + backRight.get_position() + frontLeft.get_position() + backLeft.get_position()) / 4);
  float driveSpeed = 0;
  float lastDriveSpeed = 0;
  int angleError = 0;

  //Constants here//
  float kp = 20;
  float increaseFactor = .1;

  //Tolerance Variables Here//
  int speedTolerance = 10;
  int positionTolerance = 30;

  //Deadbands//
  int speedDeadband = 2500;

  if (dir == 'b')
  {
    ticks *= -1;
  }

  while (abs(error) > positionTolerance ||
         abs(frontRight.get_actual_velocity()) > speedTolerance ||
         abs(backRight.get_actual_velocity()) > speedTolerance ||
         abs(frontLeft.get_actual_velocity()) > speedTolerance ||
         abs(backLeft.get_actual_velocity()) > speedTolerance)
  {
    angleError = startingAngle - gyro.get_value();

    error = ticks - ((frontRight.get_position() + backRight.get_position() + frontLeft.get_position() + backLeft.get_position()) / 4);

    driveSpeed = error * kp;

    if (isBetween(driveSpeed, -1 * speedDeadband, 0))
    {
      driveSpeed = -1 * speedDeadband;
    }
    if (isBetween(driveSpeed, 0, speedDeadband))
    {
      driveSpeed = speedDeadband;
    }

    setLeftDrive(driveSpeed + angleError * angleCorrectionFactor);
    setRightDrive(driveSpeed - angleError * angleCorrectionFactor);
  }
  setRightDrive(0);
  setLeftDrive(0);
}

void driveRampUp(char dir, float inches)
{

  frontRight.tare_position();
  backRight.tare_position();
  frontLeft.tare_position();
  backLeft.tare_position();

  int ticks = (int)((inches / (PI * WHEEL_RADIUS)) * 180);
  int maxAngleCorrectionFactor = 100;
  int angleCorrectionFactor = 40;
  float angleCorrectionFactorD = 2;
  int startingAngle = globalTargetAngle;

  float percentOfFullSpeed = 0;

  //P Variables Here//
  int error = ticks - ((frontRight.get_position() +
                        backRight.get_position() +
                        frontLeft.get_position() +
                        backLeft.get_position()) /
                       4);
  float driveSpeed = 0;
  float lastDriveSpeed = 0;
  int angleError = 0;
  int lastAngleError = 0;

  //Constants here//
  float kp = 20;
  float increaseFactor = .4;

  //Tolerance Variables Here//
  int speedTolerance = 10;
  int positionTolerance = 30;

  //Deadbands//
  int speedDeadband = 2500;

  if (dir == 'b')
  {
    ticks *= -1;
  }

  while (abs(error) > positionTolerance ||
         abs(frontRight.get_actual_velocity()) > speedTolerance ||
         abs(backRight.get_actual_velocity()) > speedTolerance ||
         abs(frontLeft.get_actual_velocity()) > speedTolerance ||
         abs(backLeft.get_actual_velocity()) > speedTolerance)
  {
    angleError = startingAngle - gyro.get_value();

    error = ticks - ((frontRight.get_position() + backRight.get_position() + frontLeft.get_position() + backLeft.get_position()) / 4);

    driveSpeed = error * kp;

    if (isBetween(driveSpeed, -1 * speedDeadband, 0))
    {
      driveSpeed = -1 * speedDeadband;
    }
    if (isBetween(driveSpeed, 0, speedDeadband))
    {
      driveSpeed = speedDeadband;
    }

    if (driveSpeed > 0 && lastDriveSpeed >= 0 && driveSpeed > lastDriveSpeed)
    {
      setLeftDrive(lastDriveSpeed + increaseFactor + ((angleError * angleCorrectionFactor) + ((angleError - lastAngleError) * angleCorrectionFactorD)));
      setRightDrive(lastDriveSpeed + increaseFactor - ((angleError * angleCorrectionFactor) + ((angleError - lastAngleError) * angleCorrectionFactorD)));
      lastDriveSpeed += increaseFactor;
    }
    else if (driveSpeed < 0 && lastDriveSpeed <= 0 && driveSpeed < lastDriveSpeed)
    {
      setLeftDrive(lastDriveSpeed - increaseFactor + ((angleError * angleCorrectionFactor) + ((angleError - lastAngleError) * angleCorrectionFactorD)));
      setRightDrive(lastDriveSpeed - increaseFactor - ((angleError * angleCorrectionFactor) + ((angleError - lastAngleError) * angleCorrectionFactorD)));
      lastDriveSpeed -= increaseFactor;
    }
    else
    {
      setLeftDrive(driveSpeed + ((angleError * angleCorrectionFactor) + ((angleError - lastAngleError) * angleCorrectionFactorD)));
      setRightDrive(driveSpeed - ((angleError * angleCorrectionFactor) + ((angleError - lastAngleError) * angleCorrectionFactorD)));
      lastDriveSpeed = driveSpeed;
    }
    lastAngleError = angleError;
  }
  setRightDrive(0);
  setLeftDrive(0);
}

/**
 * A function that travels a given number of inches and shoots twice during the movement
 * dir - 'f' for forwards; 'b' - for backwards
 * inches - total distance to travel in inches
 * distance1 - Distance from starting point to first shot
 * distance2 - Distance from starting point to second shot
 **/
void driveShootAsync(char dir, float inches, int distance1, int distance2)
{
  frontRight.tare_position();
  backRight.tare_position();
  frontLeft.tare_position();
  backLeft.tare_position();

  int ticks = (int)((inches / (PI * WHEEL_RADIUS)) * 180);
  int angleCorrectionFactor = 60;
  int startingAngle = globalTargetAngle;
  int traveledDistance = 0;

  //P Variables Here//
  int error = ticks - ((frontRight.get_position() + backRight.get_position() + frontLeft.get_position() + backLeft.get_position()) / 4);
  int driveSpeed = 5000;
  int angleError = 0;

  //Constants here//
  float kp = 20;
  float increaseFactor = .1;

  //Tolerance Variables Here//
  int speedTolerance = 10;
  int positionTolerance = 30;

  //Deadbands//
  int speedDeadband = 2500;

  if (dir == 'b')
  {
    ticks *= -1;
  }

  while (abs(error) > positionTolerance ||
         abs(frontRight.get_actual_velocity()) > speedTolerance ||
         abs(backRight.get_actual_velocity()) > speedTolerance ||
         abs(frontLeft.get_actual_velocity()) > speedTolerance ||
         abs(backLeft.get_actual_velocity()) > speedTolerance)
  {
    angleError = startingAngle - gyro.get_value();
    traveledDistance = ((frontRight.get_position() + backRight.get_position() + frontLeft.get_position() + backLeft.get_position()) / 4);

    error = ticks - traveledDistance;

    setLeftDrive(driveSpeed + angleError * angleCorrectionFactor);
    setRightDrive(driveSpeed - angleError * angleCorrectionFactor);
    if (isBetween(traveledDistance, distance1 - 100, distance2 + 100) || isBetween(traveledDistance, distance2 - 100, distance2 + 100))
    {
      intakeMonitor.suspend();
      indexer.move_voltage(12000);
    }
    else
    {
      indexer.move_voltage(0);
      intakeMonitor.resume();
    }
    pros::delay(5);
  }
  setRightDrive(0);
  setLeftDrive(0);
}

void turnToTarget(float targetAngle, int maxSpeed)
{
  float kp = 20;
  float scaledAngle = targetAngle * GYRO_SCALE;
  globalTargetAngle = scaledAngle * 10;
  int error = (scaledAngle * 10.0) - gyro.get_value();
  int driveSpeed = error * kp;
  int tolerance = 10;
  int speedTolerance = 5;

  while (abs(error) > tolerance ||
         frontRight.get_actual_velocity() > speedTolerance ||
         backRight.get_actual_velocity() > speedTolerance ||
         frontLeft.get_actual_velocity() > speedTolerance ||
         backLeft.get_actual_velocity() > speedTolerance)
  {
    if (isBetween(driveSpeed, -2000, 0))
    {
      driveSpeed = -2000;
    }
    if (isBetween(driveSpeed, 0, 2000))
    {
      driveSpeed = 2000;
    }

    setRightDrive(driveSpeed * -1);
    setLeftDrive(driveSpeed);

    error = (scaledAngle * 10) - gyro.get_value();
    driveSpeed = error * kp;
  }
  setRightDrive(0);
  setLeftDrive(0);
}

/**
 * This function turns a certain number of degrees using the encoders on the drive base
 * dir - The direction to turn as a character
 *   'r' will turn right
 *   'l' will turn left
 * degrees - An integer for the number of degrees the base should turn
 * topSpeed - The maximum speed the turn is permitted to move with
 * waitForCompletion - A boolean representing whether to block the program while executing
 *   true will wait for the turn to finish before moving on
 *   false will allow the program to continue moving
 */
void turn(char dir, int degrees, int topSpeed, bool waitForCompletion)
{
  double radianAngle = degreeToRadian(degrees);
  double requiredArcInches = 13.4 * radianAngle; //S in formula
  double rotationAmountPerWheel = (requiredArcInches / wheelCircumference) / 2.0;
  int encoderDegrees = (int)(rotationAmountPerWheel * 360.0);

  int fRStartingPos = frontRight.get_position();
  int bRStartingPos = backRight.get_position();
  int fLStartingPos = frontLeft.get_position();
  int bLStartingPos = backLeft.get_position();

  int tolerance = 50;

  if (dir == 'r')
  {
    frontLeft.move_relative(encoderDegrees, topSpeed);
    backLeft.move_relative(encoderDegrees, topSpeed);
    frontRight.move_relative(encoderDegrees * -1, -1 * topSpeed);
    backRight.move_relative(encoderDegrees * -1, -1 * topSpeed);
  }
  else
  {
    frontLeft.move_relative(encoderDegrees * -1, -1 * topSpeed);
    backLeft.move_relative(encoderDegrees * -1, -1 * topSpeed);
    frontRight.move_relative(encoderDegrees, topSpeed);
    backRight.move_relative(encoderDegrees, topSpeed);
  }

  if (waitForCompletion)
  {
    while (abs(frontRight.get_position() - fRStartingPos) <= abs(encoderDegrees) - tolerance || abs(backRight.get_position() - bRStartingPos) <= abs(encoderDegrees) - tolerance || abs(frontLeft.get_position() - fLStartingPos) <= abs(encoderDegrees) - tolerance || abs(backLeft.get_position() - bLStartingPos) <= abs(encoderDegrees) - 500)
    {
      pros::delay(2);
    }
  }
}

/****************************Chassis General Functions**********************************/
void holdDrivePos(int targetPosL, int targetPosR)
{
  float kp = 80;
  int tolerance = 3;
  float currentPosL = (frontLeft.get_position() + backLeft.get_position()) / 2;
  float currentPosR = (frontRight.get_position() + backRight.get_position()) / 2;
  float errorL = targetPosL - currentPosL;
  float errorR = targetPosR - currentPosR;

  if (abs(errorL) > tolerance || abs(errorR) > tolerance)
  {
    setLeftDrive(errorL * kp);
    setRightDrive(errorR * kp);
  }
  else
  {
    setLeftDrive(0);
    setRightDrive(0);
  }
}