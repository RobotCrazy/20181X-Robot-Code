#include "main.h"

#define WHEEL_RADIUS 2
#define PI atan(1) * 4
#define WHEEL_CIRCUMFERENCE WHEEL_RADIUS * 2 * pi

double pi = (atan(1) * 4);
double wheelCircumference = (WHEEL_RADIUS * 2 * pi);
double gyroScale = .78;
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

/*pros::vision_signature_s_t REDFLAG;
pros::vision_signature_s_t BLUEFLAG;
REDFLAG.id = 1;
REDFLAG.range = 3;
REDFLAG.u_min = 4681;
REDFLAG.u_max = 7621;
REDFLAG.u_mean = 6151;
REDFLAG.v_min = -1117;
REDFLAG.v_max = -429;
REDFLAG.v_mean = -773;
REDFLAG.rgb = 7160124;
REDFLAG.type = 0;
visionSensor.set_signature(1, &REDFLAG);

BLUEFLAG.id = 2;
BLUEFLAG.range = 3;
BLUEFLAG.u_min = -3085;
BLUEFLAG.u_max = 1707;
BLUEFLAG.u_mean = -2396;
BLUEFLAG.v_min = 7691;
BLUEFLAG.v_max = 11253;
BLUEFLAG.v_mean = 9472;
BLUEFLAG.rgb = 1120307;
BLUEFLAG.type = 0;
visionSensor.set_signature(2, &BLUEFLAG);

/*pros::vision_signature_s_t BLUEFLAG = {1, {1, 0, 0}, 2.600, -4321, -2115, -3218, 7633, 13239, 10436, 0, 0};
pros::vision_signature_s_t REDFLAG = {2, {1, 0, 0}, 3.400, 10269, 14613, 12441, -1509, -231, -870, 0, 0};
*/

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

  int ticks = (int)((inches / (pi * WHEEL_RADIUS)) * 180);
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

void alignToFlag()
{
  pros::vision_object_s_t flag1 = visionSensor.get_by_sig(0, 1);
  float kp = 15;
  int visionCenter = VISION_FOV_WIDTH / 2;
  int speedDeadband = 1500;
  int error = flag1.x_middle_coord - visionCenter;
  int speed = error * kp;
  pros::lcd::clear();
  while (true)
  {
    flag1 = visionSensor.get_by_sig(0, 1);
    pros::lcd::print(2, "%d", flag1.signature);
    std::cout << flag1.signature << "\n";
    int error = flag1.x_middle_coord - visionCenter;
    int speed = error * kp;

    if (speed > 0 && speed < speedDeadband)
    {
      speed = speedDeadband;
    }
    else if (speed < 0 && abs(speed) > speedDeadband)
    {
      speed = speedDeadband * -1;
    }

    /*setRightDrive(speed);
    setLeftDrive(speed * -1);*/
  }
  pros::lcd::clear();
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

  int ticks = (int)((inches / (pi * WHEEL_RADIUS)) * 180);
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

void driveV2(char dir, float inches)
{

  frontRight.tare_position();
  backRight.tare_position();
  frontLeft.tare_position();
  backLeft.tare_position();

  int ticks = (int)((inches / (pi * WHEEL_RADIUS)) * 180);
  int maxAngleCorrectionFactor = 100;
  int angleCorrectionFactor = 40;
  float angleCorrectionFactorD = 2;
  int startingAngle = globalTargetAngle;

  float percentOfFullSpeed = 0;

  //P Variables Here//
  int error = ticks - ((frontRight.get_position() + backRight.get_position() + frontLeft.get_position() + backLeft.get_position()) / 4);
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
      std::cout << "First if\n";
      setLeftDrive(lastDriveSpeed + increaseFactor + ((angleError * angleCorrectionFactor) + ((angleError - lastAngleError) * angleCorrectionFactorD)));
      setRightDrive(lastDriveSpeed + increaseFactor - ((angleError * angleCorrectionFactor) + ((angleError - lastAngleError) * angleCorrectionFactorD)));
      lastDriveSpeed += increaseFactor;
    }
    else if (driveSpeed < 0 && lastDriveSpeed <= 0 && driveSpeed < lastDriveSpeed)
    {
      std::cout << "Second if\n";
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

    //percentOfFullSpeed = driveSpeed / 12000.0;
    //angleCorrectionFactor = maxAngleCorrectionFactor * (1.0 - percentOfFullSpeed);
    //This is an alternate solution.  Try this if the D term does not work, or try it with the D term.
    lastAngleError = angleError;
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

void setFlywheelVoltage(int voltage)
{
  flywheel.move_voltage(voltage);
}
void startFlywheelVoltage(int voltage)
{
  maintainFlywheelSpeedRequested = false;
  setFlywheelVoltage(voltage);
}
void startFlywheel(int targetSpeed)
{
  maintainFlywheelSpeedRequested = true;
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

void setFlywheelTargetSpeed(int speed)
{
  targetFlywheelSpeed = speed;
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

/**
 * This function shoots the ball by spinning the indexer once the flywheel velocity
 * is greater than or equal to the desired speed
 * requiredSpeed - The required speed at which the flywheel must shoot
 * intakeTicks - The number of ticks that the indexer must rotate to shoot the ball
 * stopFlywheelOnFinish - A boolean representing whether to shut off the flywheel after
 * shooting
 *    Pass true to shut off flywheel after shooting
 *    Pass false to allow the flywheel to continue running
 **/
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

void setGlobalTargetAngle(int newAngle)
{
  globalTargetAngle = (newAngle * gyroScale * 10);
}

void turnToTarget(float targetAngle, int maxSpeed)
{
  float kp = 20;
  float scaledAngle = targetAngle * gyroScale;
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

void testAuto()
{
  //startIntake();
  /*startFlywheel(120);
  pros::delay(12000);
  shootWhenReady(120, 600, false);*/
  //Programming Skills Routine Follows://
  startIntakeOut();
  drive('f', 33);
  startIntake();
  drive('f', 6);
  driveV2('b', 5);
  stopIntake();
  startFlywheel(190);
  driveV2('b', 34);
  turnToTarget(-89, 100);
  driveV2('f', 56.5);
  shootWhenReady(180, 500, false);
  driveV2('f', 20.5);
  shootWhenReady(180, 800, true);
  setGlobalTargetAngle(-98);
  driveV2('f', 18);
  setGlobalTargetAngle(-90);
  driveV2('b', 48);
  turnToTarget(0, 100);
}
void autoOriginal() //Blue Front Original
{
}
void auto1() //Blue Front
{
  startFlywheel(190);
  startIntake();
  drive('f', 35);
  drive('b', 5);
  stopIntake();
  drive('b', 31);
  turnToTarget(88, 100);
  drive('f', 10);
  shootWhenReady(180, 1000, false);
  drive('f', 19);
  shootWhenReady(180, 1000, true);
  setGlobalTargetAngle(98);
  drive('f', 18.5);
  drive('b', 27);
  startIntakeOut();
  turnToTarget(0, 100);
  drive('f', 25);
  stopIntake();
  turnToTarget(47, 100);
  drive('f', 30);
}

void auto2() //Blue Front Shoot first
{
}

void auto3() //Blue Back
{
}

/*void auto4() //Red Front
{
  startFlywheel(190);
  startIntake();
  drive('f', 35);
  drive('b', 5);
  stopIntake();
  drive('b', 31);
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
}*/

void auto4() //Red Front
{
  startFlywheel(190);
  startIntake();
  drive('f', 35);
  drive('b', 5);
  stopIntake();
  drive('b', 31);
  turnToTarget(-88, 100);
  drive('f', 10);
  shootWhenReady(180, 1000, false);
  drive('f', 19);
  shootWhenReady(180, 1000, true);
  setGlobalTargetAngle(-98);
  drive('f', 18.5);
  drive('b', 27);
  startIntakeOut();
  turnToTarget(0, 100);
  drive('f', 25);
  stopIntake();
  turnToTarget(-47, 100);
  drive('f', 30);
}
void auto5() //Red Front Shoot First
{
}
void auto6() //Red Back
{
  startFlywheel(11200, 190);
  startIntake();
  drive('f', 35);
  drive('b', 5);
  stopIntake();
  drive('b', 16);
  turnToTarget(-71, 100);
  shootWhenReady(190, 600, false);
  pros::delay(700);
  setFlywheelVoltage(9000);
  setFlywheelTargetSpeed(170);
  shootWhenReady(170, 500, true);
  turnToTarget(40, 100);
  startIntakeOut();
  drive('f', 35);
}

void autonomous()
{

  //pros::Task flywheelRPMMonitor(maintainFlywheelSpeed, parameter3, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Flywheel speed task");
  //pros::Task intakeMonitor(monitorIntake, parameter2, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Intake auto movement task");

  autoMode = 1;
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
