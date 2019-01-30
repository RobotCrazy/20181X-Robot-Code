#include "main.h"

#define WHEEL_RADIUS 2
#define PI atan(1) * 4
#define WHEEL_CIRCUMFERENCE WHEEL_RADIUS * 2 * pi

double pi = (atan(1) * 4);
double wheelCircumference = (WHEEL_RADIUS * 2 * pi);
int globalTargetAngle = 0;

double degreeToRadian(double degrees)
{
  return degrees * (pi / 180.0);
}

bool isBetween(float number, float rangeLower, float rangeUpper)
{
  return (number > rangeLower && number < rangeUpper);
}

void setRightDrive(int voltage)
{
  if (voltage == 0)
  {
    frontRight.move_voltage(0);
    backRight.move_voltage(0);
    frontRight.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
    backRight.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
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
    frontLeft.move_voltage(0);
    frontLeft.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
    backLeft.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
  }
  else
  {
    frontLeft.move_voltage(voltage);
    backLeft.move_voltage(voltage);
  }
}

void gyroscopeFiltering()
{
  while (true)
  {

    pros::delay(2);
  }
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

  int ticks = (int)((inches / (pi * WHEEL_RADIUS)) * 180);
  int angleCorrectionFactor = 60;
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

    /*if (driveSpeed > 0 && driveSpeed > lastDriveSpeed)
    {
      float currentSpeed = lastDriveSpeed + increaseFactor;
      setLeftDrive(currentSpeed);
      setRightDrive(currentSpeed);
      lastDriveSpeed = currentSpeed;
    }
    else if (driveSpeed < 0 && driveSpeed < lastDriveSpeed)
    {
      float currentSpeed = lastDriveSpeed - increaseFactor;
      setLeftDrive(currentSpeed);
      setRightDrive(currentSpeed);
      lastDriveSpeed = currentSpeed;
    }
    else
    {
      setLeftDrive(driveSpeed);
      setRightDrive(driveSpeed);
      lastDriveSpeed = driveSpeed;
    }*/
    setLeftDrive(driveSpeed + angleError * angleCorrectionFactor);
    setRightDrive(driveSpeed - angleError * angleCorrectionFactor);
  }
  setRightDrive(0);
  setLeftDrive(0);
}

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

void startFlywheel(int targetSpeed)
{
  maintainFlywheelSpeedRequested = true;
  targetFlywheelSpeed = targetSpeed;
}

void stopFlywheel()
{
  maintainFlywheelSpeedRequested = false;
  targetFlywheelSpeed = 0;
}

void startIntake()
{
  intakeUpRequested = true;
  prepareShotRequested = false;
  shootBallRequested = false;
  intakeOutRequested = false;
}

void startIntakeOut()
{
  intakeUpRequested = false;
  prepareShotRequested = false;
  shootBallRequested = false;
  intakeOutRequested = true;
}

void stopIntake()
{
  intakeUpRequested = false;
  prepareShotRequested = true;
  shootBallRequested = false;
  intakeOutRequested = false;
}

void prepareShot()
{
  while (!(isBetween(indexerSonar.get_value(), 50, 80)))
  {
    intake.move_velocity(200);
    indexer.move_velocity(200);
  }
  intake.move_velocity(0);
  indexer.move_velocity(0);
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
  intakeMonitor.suspend();
  indexer.move_relative(intakeTicks, 200);
  pros::delay(500);
  intakeMonitor.resume();
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
  float kp = 20;
  float scaledAngle = targetAngle * .78;
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

/*vision::signature BLUEFLAG(1, -4321, -2115, -3218, 7633, 13239, 10436, 2.600, 0);
vision::signature REDFLAG(2, 10269, 14613, 12441, -1509, -231, -870, 3.400, 0);
vision::signature SIG_3(3, 0, 0, 0, 0, 0, 0, 3.000, 0);
vision::signature SIG_4(4, 0, 0, 0, 0, 0, 0, 3.000, 0);
vision::signature SIG_5(5, 0, 0, 0, 0, 0, 0, 3.000, 0);
vision::signature SIG_6(6, 0, 0, 0, 0, 0, 0, 3.000, 0);
vision::signature SIG_7(7, 0, 0, 0, 0, 0, 0, 3.000, 0);
vex::vision vision1(vex::PORT1, 50, BLUEFLAG, REDFLAG, SIG_3, SIG_4, SIG_5, SIG_6, SIG_7);*/

/*pros::vision_signature_s_t BLUEFLAG = {1, {1, 0, 0}, 2.600, -4321, -2115, -3218, 7633, 13239, 10436, 0, 0};
pros::vision_signature_s_t REDFLAG = {2, {1, 0, 0}, 3.400, 10269, 14613, 12441, -1509, -231, -870, 0, 0};
pros::vision_signature_s_t SIG_3 = {3, {1, 0, 0}, 3.000, 0, 0, 0, 0, 0, 0, 0, 0};
pros::vision_signature_s_t SIG_4 = {4, {1, 0, 0}, 3.000, 0, 0, 0, 0, 0, 0, 0, 0};
pros::vision_signature_s_t SIG_5 = {5, {1, 0, 0}, 3.000, 0, 0, 0, 0, 0, 0, 0, 0};
pros::vision_signature_s_t SIG_6 = {6, {1, 0, 0}, 3.000, 0, 0, 0, 0, 0, 0, 0, 0};
pros::vision_signature_s_t SIG_7 = {7, {1, 0, 0}, 3.000, 0, 0, 0, 0, 0, 0, 0, 0};
*/
void testAuto()
{
  startIntakeOut();
  drive('f', 33);
  startIntake();
  drive('f', 6);
  drive('b', 5);
  stopIntake();
  startFlywheel(190);
  drive('b', 34);
  turnToTarget(-88, 100);
  drive('f', 60);
  shootWhenReady(180, 500, false);
  drive('f', 19);
  shootWhenReady(180, 800, true);
  turnToTarget(75, 100);
  drive('b', 15);
  drive('f', 35);
  turnToTarget(0, 100);
}
void autoOriginal() //Blue Front Original
{
}
void auto1() //Blue Front
{
}

void auto2() //Blue Front Shoot first
{
}

void auto3() //Blue Back
{
}

void auto4() //Red Front
{
  startFlywheel(190);
  startIntake();
  drive('f', 35);
  drive('b', 5);
  stopIntake();
  drive('b', 32);
  turnToTarget(-88, 100);
  drive('f', 10);
  shootWhenReady(180, 1000, false);
  drive('f', 19);
  shootWhenReady(180, 1000, true);
  turnToTarget(-98, 100);
  drive('f', 18);
  drive('b', 25);
  startIntakeOut();
  turnToTarget(0, 100);
  drive('f', 41);
  turnToTarget(90, 100);
  drive('b', 30);
}
void auto5() //Red Front Shoot First
{
}
void auto6() //Red Back
{
}

void autonomous()
{

  //pros::Task flywheelRPMMonitor(maintainFlywheelSpeed, parameter3, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Flywheel speed task");
  //pros::Task intakeMonitor(monitorIntake, parameter2, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Intake auto movement task");

  autoMode = 4;
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
  /*flywheelRPMMonitor.suspend();
  intakeMonitor.suspend();*/

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
