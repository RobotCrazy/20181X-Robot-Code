#include "main.h"

#define WHEEL_RADIUS 2
#define PI atan(1) * 4
#define WHEEL_CIRCUMFERENCE WHEEL_RADIUS * 2 * pi

double pi = (atan(1) * 4);
double wheelCircumference = (WHEEL_RADIUS * 2 * pi);

double degreeToRadian(double degrees)
{
  return degrees * (pi / 180.0);
}

bool isBetween(float number, float rangeLower, float rangeUpper)
{
  return (number > rangeLower && number < rangeUpper);
}

/**
 * Use this function to drive for a certain number of inches
 * minSpeed - The minimum value the drive base can go at
 * Note: Set minSpeed to 0 to leave the deadband at its default
 **/
void drive(char dir, float inches, int topSpeed, int minSpeed)
{

  int ticks = (int)((inches / (pi * WHEEL_RADIUS)) * 180);

  frontRight.tare_position();
  backRight.tare_position();
  frontLeft.tare_position();
  backLeft.tare_position();

  float kp = 20;
  double ki = .0003;
  float kd = 0;

  int integralZone = 150;
  float driveSpeedR = 0;
  float driveSpeedL = 0;
  float lastErrorL = 0;
  float lastErrorR = 0;
  float totalErrorL = 0;
  float totalErrorR = 0;
  float lastDriveSpeedL = 0;
  float lastDriveSpeedR = 0;

  float proportionL = 0;
  float proportionR = 0;
  double integralL = 0;
  double integralR = 0;
  float derivitiveL = 0;
  float derivitiveR = 0;

  int tolerance = 80;
  int speedTolerance = 10;
  float increaseFactor = .05;
  float speedDeadband = 0;
  /*if (minSpeed != 0)
  {
    speedDeadband = minSpeed;
  }*/

  if (dir == 'f')
  {

    int errorR = ticks - frontRight.get_position();
    int errorL = ticks - frontLeft.get_position();

    while (abs(errorR) > tolerance || abs(errorL) > tolerance || frontRight.get_actual_velocity() > speedTolerance || backRight.get_actual_velocity() > speedTolerance || frontLeft.get_actual_velocity() > speedTolerance || backLeft.get_actual_velocity() > speedTolerance)
    {

      errorR = (ticks - frontRight.get_position());
      errorL = (ticks - frontLeft.get_position());

      proportionL = errorL * kp;
      proportionR = errorR * kp;

      if (errorL == 0)
      {
        derivitiveL = 0;
      }
      else
      {
        derivitiveL = (errorL - lastErrorL) * kd;
      }

      if (errorR == 0)
      {
        derivitiveR = 0;
      }
      else
      {
        derivitiveR = (errorR - lastErrorR) * kd;
      }

      integralL = totalErrorL * ki;
      integralR = totalErrorR * ki;

      if (abs(errorL) < integralZone && errorL != 0)
      {
        totalErrorL += errorL;
      }
      else
      {
        integralL = 0;
        totalErrorL = 0;
      }

      if (abs(errorR) < integralZone && errorR != 0)
      {
        totalErrorR += errorR;
      }
      else
      {
        integralR = 0;
        totalErrorR = 0;
      }

      driveSpeedR = (proportionR + derivitiveR + integralR);
      driveSpeedL = (proportionL + derivitiveL + integralL);

      /*if (isBetween(driveSpeedL, -1 * speedDeadband, 0))
        {
          driveSpeedL = (-1 * speedDeadband);
        }
        else if (isBetween(driveSpeedL, 0, speedDeadband))
        {
          driveSpeedL = speedDeadband;
        }

        if (isBetween(driveSpeedR, -1 * speedDeadband, 0))
        {
          driveSpeedR = (-1 * speedDeadband);
        }
        else if (isBetween(driveSpeedR, 0, speedDeadband))
        {
          driveSpeedR = speedDeadband;
        }*/

      if (driveSpeedL > 0 && driveSpeedL >= speedDeadband && driveSpeedL > lastDriveSpeedL)
      {
        float currentSpeedL = lastDriveSpeedL + increaseFactor;
        frontLeft.move_voltage(currentSpeedL);
        backLeft.move_voltage(currentSpeedL);
        lastDriveSpeedL = currentSpeedL;
      }
      else if (driveSpeedL < 0 && driveSpeedL <= speedDeadband && driveSpeedL < lastDriveSpeedL)
      {
        float currentSpeedL = lastDriveSpeedL - increaseFactor;
        frontLeft.move_voltage(currentSpeedL);
        backLeft.move_voltage(currentSpeedL);
        lastDriveSpeedL = currentSpeedL;
      }
      else
      {
        frontLeft.move_voltage(driveSpeedL);
        backLeft.move_voltage(driveSpeedL);
        lastDriveSpeedL = driveSpeedL;
      }

      if (driveSpeedR > 0 && driveSpeedR >= speedDeadband && driveSpeedR > lastDriveSpeedR)
      {
        float currentSpeedR = lastDriveSpeedR + increaseFactor;
        frontRight.move_voltage(currentSpeedR);
        backRight.move_voltage(currentSpeedR);
        lastDriveSpeedR = currentSpeedR;
      }
      else if (driveSpeedR < 0 && driveSpeedR <= speedDeadband && driveSpeedR < lastDriveSpeedR)
      {
        float currentSpeedR = lastDriveSpeedR - increaseFactor;
        frontRight.move_voltage(currentSpeedR);
        backRight.move_voltage(currentSpeedR);
        lastDriveSpeedR = currentSpeedR;
      }
      else
      {
        frontRight.move_voltage(driveSpeedR);
        backRight.move_voltage(driveSpeedR);
        lastDriveSpeedR = driveSpeedR;
      }

      lastErrorL = errorL;
      lastErrorR = errorR;
    }
    frontRight.move_voltage(0);
    backRight.move_voltage(0);
    frontLeft.move_voltage(0);
    backLeft.move_voltage(0);
  }
  else if (dir == 'b')
  {
    kp *= -1;
    int errorR = (ticks - (frontRight.get_position() * -1));
    int errorL = (ticks - (frontLeft.get_position() * -1));
    float driveSpeedR = errorR * kp;
    float driveSpeedL = errorL * kp;
    while (abs(errorR) > tolerance || abs(errorL) > tolerance || frontRight.get_actual_velocity() > speedTolerance || backRight.get_actual_velocity() > speedTolerance || frontLeft.get_actual_velocity() > speedTolerance || backLeft.get_actual_velocity() > speedTolerance)
    {
      {

        errorR = (ticks - (frontRight.get_position() * -1));
        errorL = (ticks - (frontLeft.get_position() * -1));
        driveSpeedR = errorR * kp;
        driveSpeedL = errorL * kp;

        if (isBetween(driveSpeedL, -1 * speedDeadband, 0))
        {
          driveSpeedL = (-1 * speedDeadband);
        }
        else if (isBetween(driveSpeedL, 0, speedDeadband))
        {
          driveSpeedL = speedDeadband;
        }

        if (isBetween(driveSpeedR, -1 * speedDeadband, 0))
        {
          driveSpeedR = (-1 * speedDeadband);
        }
        else if (isBetween(driveSpeedR, 0, speedDeadband))
        {
          driveSpeedR = speedDeadband;
        }

        if (driveSpeedL > 0 && driveSpeedL > lastDriveSpeedL)
        {
          float currentSpeedL = lastDriveSpeedL + increaseFactor;
          frontLeft.move_voltage(currentSpeedL);
          backLeft.move_voltage(currentSpeedL);
          lastDriveSpeedL = currentSpeedL;
        }
        else if (driveSpeedL < 0 && driveSpeedL < lastDriveSpeedL)
        {
          float currentSpeedL = lastDriveSpeedL - increaseFactor;
          frontLeft.move_voltage(currentSpeedL);
          backLeft.move_voltage(currentSpeedL);
          lastDriveSpeedL = currentSpeedL;
        }
        else
        {
          frontLeft.move_voltage(driveSpeedL);
          backLeft.move_voltage(driveSpeedL);
          lastDriveSpeedL = driveSpeedL;
        }

        if (driveSpeedR > 0 && driveSpeedR > lastDriveSpeedR)
        {
          float currentSpeedR = lastDriveSpeedR + increaseFactor;
          frontRight.move_voltage(currentSpeedR);
          backRight.move_voltage(currentSpeedR);
          lastDriveSpeedR = currentSpeedR;
        }
        else if (driveSpeedR < 0 && driveSpeedR < lastDriveSpeedR)
        {
          float currentSpeedR = lastDriveSpeedR - increaseFactor;
          frontRight.move_voltage(currentSpeedR);
          backRight.move_voltage(currentSpeedR);
          lastDriveSpeedR = currentSpeedR;
        }
        else
        {
          frontRight.move_voltage(driveSpeedR);
          backRight.move_voltage(driveSpeedR);
          lastDriveSpeedR = driveSpeedR;
        }
      }
    }
    frontRight.move_voltage(0);
    backRight.move_voltage(0);
    frontLeft.move_voltage(0);
    backLeft.move_voltage(0);
  }
}

/*void driveWithEndJerk(char dir, float inches, int topSpeed, int minSpeed, float jerkAmount)
{

  int ticks = (int)((inches / (pi * WHEEL_RADIUS)) * 180);
  int jerkTicks = (int)((jerkAmount / (pi * WHEEL_RADIUS)) * 180);

  frontRight.tare_position();
  backRight.tare_position();
  frontLeft.tare_position();
  backLeft.tare_position();
  pros::lcd::print(0, "%d", frontRight.get_position());

  float kp = 15;
  float lastDriveSpeedL = 0;
  float lastDriveSpeedR = 0;
  int tolerance = 80;
  int speedTolerance = 10;
  float increaseFactor = .3;
  float speedDeadband = 2500;

  if (dir == 'f')
  {
    int errorR = ticks - frontRight.get_position();
    int errorL = ticks - frontLeft.get_position();
    float driveSpeedR = errorR * kp;
    float driveSpeedL = errorL * kp;
    while (abs(errorR) > tolerance || abs(errorL) > tolerance || frontRight.get_actual_velocity() > speedTolerance || backRight.get_actual_velocity() > speedTolerance || frontLeft.get_actual_velocity() > speedTolerance || backLeft.get_actual_velocity() > speedTolerance)
    {
      {

        errorR = (ticks - frontRight.get_position());
        errorL = (ticks - frontLeft.get_position());
        driveSpeedR = errorR * kp;
        driveSpeedL = errorL * kp;
        if (errorR < jerkTicks && errorL < jerkTicks)
        {
          speedDeadband = minSpeed;
        }

        if (isBetween(driveSpeedL, -1 * speedDeadband, 0))
        {
          driveSpeedL = (-1 * speedDeadband);
        }
        else if (isBetween(driveSpeedL, 0, speedDeadband))
        {
          driveSpeedL = speedDeadband;
        }

        if (isBetween(driveSpeedR, -1 * speedDeadband, 0))
        {
          driveSpeedR = (-1 * speedDeadband);
        }
        else if (isBetween(driveSpeedR, 0, speedDeadband))
        {
          driveSpeedR = speedDeadband;
        }

        if (driveSpeedL > 0 && driveSpeedL >= speedDeadband && driveSpeedL > lastDriveSpeedL)
        {
          float currentSpeedL = lastDriveSpeedL + increaseFactor;
          frontLeft.move_voltage(currentSpeedL);
          backLeft.move_voltage(currentSpeedL);
          lastDriveSpeedL = currentSpeedL;
        }
        else if (driveSpeedL < 0 && driveSpeedL <= speedDeadband && driveSpeedL < lastDriveSpeedL)
        {
          float currentSpeedL = lastDriveSpeedL - increaseFactor;
          frontLeft.move_voltage(currentSpeedL);
          backLeft.move_voltage(currentSpeedL);
          lastDriveSpeedL = currentSpeedL;
        }
        else
        {
          frontLeft.move_voltage(driveSpeedL);
          backLeft.move_voltage(driveSpeedL);
          lastDriveSpeedL = driveSpeedL;
        }

        if (driveSpeedR > 0 && driveSpeedR >= speedDeadband && driveSpeedR > lastDriveSpeedR)
        {
          float currentSpeedR = lastDriveSpeedR + increaseFactor;
          frontRight.move_voltage(currentSpeedR);
          backRight.move_voltage(currentSpeedR);
          lastDriveSpeedR = currentSpeedR;
        }
        else if (driveSpeedR < 0 && driveSpeedR <= speedDeadband && driveSpeedR < lastDriveSpeedR)
        {
          float currentSpeedR = lastDriveSpeedR - increaseFactor;
          frontRight.move_voltage(currentSpeedR);
          backRight.move_voltage(currentSpeedR);
          lastDriveSpeedR = currentSpeedR;
        }
        else
        {
          frontRight.move_voltage(driveSpeedR);
          backRight.move_voltage(driveSpeedR);
          lastDriveSpeedR = driveSpeedR;
        }
      }
    }
    frontRight.move_voltage(0);
    backRight.move_voltage(0);
    frontLeft.move_voltage(0);
    backLeft.move_voltage(0);
  }
  else if (dir == 'b')
  {
    kp *= -1;
    int errorR = (ticks - (frontRight.get_position() * -1));
    int errorL = (ticks - (frontLeft.get_position() * -1));
    float driveSpeedR = errorR * kp;
    float driveSpeedL = errorL * kp;
    while (abs(errorR) > tolerance || abs(errorL) > tolerance || frontRight.get_actual_velocity() > speedTolerance || backRight.get_actual_velocity() > speedTolerance || frontLeft.get_actual_velocity() > speedTolerance || backLeft.get_actual_velocity() > speedTolerance)
    {
      {

        errorR = (ticks - (frontRight.get_position() * -1));
        errorL = (ticks - (frontLeft.get_position() * -1));
        driveSpeedR = errorR * kp;
        driveSpeedL = errorL * kp;
        if (errorR < jerkTicks && errorL < jerkTicks)
        {
          speedDeadband = minSpeed;
        }

        if (isBetween(driveSpeedL, -1 * speedDeadband, 0))
        {
          driveSpeedL = (-1 * speedDeadband);
        }
        else if (isBetween(driveSpeedL, 0, speedDeadband))
        {
          driveSpeedL = speedDeadband;
        }

        if (isBetween(driveSpeedR, -1 * speedDeadband, 0))
        {
          driveSpeedR = (-1 * speedDeadband);
        }
        else if (isBetween(driveSpeedR, 0, speedDeadband))
        {
          driveSpeedR = speedDeadband;
        }

        if (driveSpeedL > 0 && driveSpeedL > lastDriveSpeedL)
        {
          float currentSpeedL = lastDriveSpeedL + increaseFactor;
          frontLeft.move_voltage(currentSpeedL);
          backLeft.move_voltage(currentSpeedL);
          lastDriveSpeedL = currentSpeedL;
        }
        else if (driveSpeedL < 0 && driveSpeedL < lastDriveSpeedL)
        {
          float currentSpeedL = lastDriveSpeedL - increaseFactor;
          frontLeft.move_voltage(currentSpeedL);
          backLeft.move_voltage(currentSpeedL);
          lastDriveSpeedL = currentSpeedL;
        }
        else
        {
          frontLeft.move_voltage(driveSpeedL);
          backLeft.move_voltage(driveSpeedL);
          lastDriveSpeedL = driveSpeedL;
        }

        if (driveSpeedR > 0 && driveSpeedR > lastDriveSpeedR)
        {
          float currentSpeedR = lastDriveSpeedR + increaseFactor;
          frontRight.move_voltage(currentSpeedR);
          backRight.move_voltage(currentSpeedR);
          lastDriveSpeedR = currentSpeedR;
        }
        else if (driveSpeedR < 0 && driveSpeedR < lastDriveSpeedR)
        {
          float currentSpeedR = lastDriveSpeedR - increaseFactor;
          frontRight.move_voltage(currentSpeedR);
          backRight.move_voltage(currentSpeedR);
          lastDriveSpeedR = currentSpeedR;
        }
        else
        {
          frontRight.move_voltage(driveSpeedR);
          backRight.move_voltage(driveSpeedR);
          lastDriveSpeedR = driveSpeedR;
        }
      }
    }
    frontRight.move_voltage(0);
    backRight.move_voltage(0);
    frontLeft.move_voltage(0);
    backLeft.move_voltage(0);
  }
}*/

void driveTime(char dir, int milliseconds, int topSpeed)
{
  int currentTime = pros::millis();
  if (dir == 'b')
  {
    topSpeed *= -1;
  }

  frontRight.move_voltage(topSpeed);
  backRight.move_voltage(topSpeed);
  frontLeft.move_voltage(topSpeed);
  backLeft.move_voltage(topSpeed);

  while (abs(pros::millis() - currentTime) < milliseconds)
  {
    pros::delay(2);
  }
  frontRight.move_voltage(0);
  backRight.move_voltage(0);
  frontLeft.move_voltage(0);
  backLeft.move_voltage(0);
}

void startFlywheel(int voltage)
{
  flywheel.move_voltage(voltage);
}

void stopFlywheel()
{
  flywheel.move_voltage(0);
}

void startIntake()
{
  intake.move_velocity(-200);
}

void stopIntake()
{
  intake.move_velocity(0);
}

void runIntake(char dir, int ticks, bool waitForCompletion)
{
  if (dir == 'u')
  { //u is up, d is down
    ticks *= -1;
  }
  int originalPos = intake.get_position();
  intake.move_relative(ticks, 200);
  if (waitForCompletion)
  {
    while (abs(intake.get_position() - originalPos) < abs(ticks) - 5)
    {
      pros::delay(2);
    }
  }
}

void shootWhenReady(int requiredSpeed, int intakeTicks, bool stopFlywheelOnFinish)
{
  while (flywheel.get_actual_velocity() < requiredSpeed)
  {
    pros::delay(2);
  }
  runIntake('u', intakeTicks, true);
  pros::delay(500);
  if (stopFlywheelOnFinish)
  {
    stopFlywheel();
  }
}

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

void turnToTarget(float targetAngle, int maxSpeed)
{
  float kp = 12;
  float scaledAngle = targetAngle * .75;
  int error = (scaledAngle * 10.0) - gyro.get_value();
  int driveSpeed = error * kp;
  int tolerance = 10;
  int speedTolerance = 5;

  while (abs(error) > tolerance || frontRight.get_actual_velocity() > speedTolerance || backRight.get_actual_velocity() > speedTolerance || frontLeft.get_actual_velocity() > speedTolerance || backLeft.get_actual_velocity() > speedTolerance)
  {
    if (isBetween(driveSpeed, -2000, 0))
    {
      driveSpeed = -2000;
    }
    if (isBetween(driveSpeed, 0, 2000))
    {
      driveSpeed = 2000;
    }

    frontRight.move_voltage(driveSpeed * -1);
    backRight.move_voltage(driveSpeed * -1);
    frontLeft.move_voltage(driveSpeed);
    backLeft.move_voltage(driveSpeed);

    error = (scaledAngle * 10) - gyro.get_value();
    driveSpeed = error * kp;
  }
  frontRight.move_voltage(0);
  backRight.move_voltage(0);
  frontLeft.move_voltage(0);
  backLeft.move_voltage(0);
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void testAuto()
{
  startIntake();
  drive('f', 41, 150, 0);
  stopIntake();
}
void autoOriginal() //Blue Front Original
{
  startFlywheel(10000);
  shootWhenReady(170, 700, true);
  drive('b', 6, 50, true);
  turnToTarget(64.5, 100);
  drive('f', 41, 150, true);
  runIntake('u', 800, true);
  startFlywheel(10500);
  drive('b', 36.5, 150, true);
  turnToTarget(0, 100);
  drive('f', 29, 90, true);
  turnToTarget(0, 75);
  shootWhenReady(160, 700, true);
  //turnToTarget(-10, 60);
  drive('f', 30, 200, true);
}
void auto1() //Blue Front
{
  startFlywheel(10500);
  drive('f', 40, 150, 4000);
  runIntake('u', 800, true);
  drive('b', 42, 150, 0);
  turnToTarget(90, 100);
  shootWhenReady(165, 400, false);
  drive('f', 30, 50, 0);
  turnToTarget(86, 100);
  shootWhenReady(165, 600, true);
  drive('f', 25, 100, 0);
}

void auto2()
{ //Blue Front Shoot first
  startFlywheel(10000);
  shootWhenReady(170, 700, true);
  drive('b', 6, 50, true);
  turnToTarget(-90, 100);
  drive('f', 41, 150, true);
  runIntake('u', 800, true);
  startFlywheel(10500);
  drive('b', 36.5, 150, true);
  turnToTarget(3, 100);
  drive('f', 29, 90, true);
  turnToTarget(3, 75);
  shootWhenReady(160, 700, true);
  //turnToTarget(-10, 60);
  drive('f', 30, 200, true);
}

void auto3() //Blue Back
{
  startFlywheel(11200);
  drive('f', 40, 150, 5000);
  runIntake('u', 800, true);
  drive('b', 38, 150, 0);
  //turnToTarget(-45, 100);
  turnToTarget(65.5, 100);
  shootWhenReady(188, 400, false);
  pros::delay(250);
  shootWhenReady(170, 600, true);
  startIntake();
  turnToTarget(90, 100);
  drive('f', 20, 100, 0);
  stopIntake();
  turnToTarget(180, 100);
  drive('b', 50, 200, 0);
}

void auto4() //Red Front
{
  startFlywheel(10500);
  drive('f', 40, 150, 4000);
  runIntake('u', 800, true);
  drive('b', 42, 150, 0);
  turnToTarget(-90, 100);
  shootWhenReady(165, 400, false);
  drive('f', 30, 50, 0);
  turnToTarget(-86, 100);
  shootWhenReady(165, 600, true);
  drive('f', 25, 100, 0);
}
void auto5()
{ //Red Front Shoot First
  startFlywheel(10000);
  shootWhenReady(170, 700, true);
  drive('b', 6, 50, true);
  turnToTarget(90, 100);
  drive('f', 41, 150, true);
  runIntake('u', 800, true);
  startFlywheel(10500);
  drive('b', 36.5, 150, true);
  turnToTarget(-3, 100);
  drive('f', 29, 90, true);
  turnToTarget(-3, 75);
  shootWhenReady(160, 700, true);
  //turnToTarget(-10, 60);
  drive('f', 30, 200, true);
}
void auto6() //Red Back
{
  startFlywheel(11200);
  drive('f', 40, 150, 5000);
  runIntake('u', 800, true);
  drive('b', 38, 150, 0);
  //turnToTarget(-45, 100);
  turnToTarget(-65.5, 100);
  shootWhenReady(188, 400, false);
  pros::delay(250);
  shootWhenReady(170, 600, true);
  startIntake();
  turnToTarget(-90, 100);
  drive('f', 20, 100, 0);
  stopIntake();
  turnToTarget(-180, 100);
  drive('b', 50, 200, 0);
}
void autonomous()
{
  autoMode = 100;
  if (autoMode == 1)
  {
    auto1();
  }
  else if (autoMode == 2)
  {
    auto2();
  }
  else if (autoMode == 3)
  {
    auto3();
  }
  else if (autoMode == 4)
  {
    auto4();
  }
  else if (autoMode == 5)
  {
    auto5();
  }
  else if (autoMode == 6)
  {
    auto6();
  }
  testAuto();

  /*startFlywheel(10000);
  drive('f', 41, 150, true);
  runIntake('u', 550, true);
  drive('b', 35, 150, true);
  turnToTarget(-64.5, 100);
  shootWhenReady(175, 400);
  pros::delay(250);
  shootWhenReady(150, 600);
  drive('f', 18, 100, true);
  turnToTarget(-140, 100);
  drive('b', 55, 200, true);
  */

  //-70 is a complete turn around for the platform

  /*startFlywheel(10000);
  drive('f', 41, 150, true);
  runIntake('u', 1000, true);
  drive('b', 35, 150, true);
  turn('l', 105.5, 100, true);
  drive('b', 30, 125, true);
  shootWhenReady(180, 900);
  drive('f', 44, 150, true);
  turn('l', 105.5, 50, true);
  drive('b', 55, 200, true);*/
  //drive('f', )
  //runIntake('u', 1000, true);
  //turn('r', 90, true);
}
