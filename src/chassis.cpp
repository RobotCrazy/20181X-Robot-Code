#include "chassis.h"
#include "mathUtil.h"

pros::Motor frontLeft(FRONT_LEFT_PORT);
pros::Motor backLeft(BACK_LEFT_PORT);
pros::Motor frontRight(FRONT_RIGHT_PORT, true);
pros::Motor backRight(BACK_RIGHT_PORT, true);
pros::ADIGyro gyro(GYRO_PORT);

/*****************************Chassis Movement Global Variables************************************/
int globalTargetAngle = 0;

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