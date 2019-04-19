#include "main.h"

//double gyroScale = .78;

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

/*void testAuto()
{
  //startIntake();
  /*startFlywheel(120);
  pros::delay(12000);
  shootWhenReady(120, 600, false);*/
//Programming Skills Routine Follows://
/*startIntakeOut();
  driveRampUp('f', 33);
  startIntake();
  drive('f', 6);
  driveRampUp('b', 5);
  stopIntake();
  startFlywheel(190);
  driveRampUp('b', 34);
  turnToTarget(-89, 100);
  driveRampUp('f', 56.5);
  shootWhenReady(180, 500, false);
  driveRampUp('f', 20.5);
  shootWhenReady(180, 800, true);
  setGlobalTargetAngle(-99);
  driveRampUp('f', 18);
  setGlobalTargetAngle(-90);
  driveRampUp('b', 48);
  turnToTarget(0, 100);
  startIntakeOut();
  startFlywheel(180);
  driveRampUp('f', 35);
  startIntake();
  drive('f', 6);
  driveRampUp('b', 8);
  turnToTarget(-73, 100);
  driveRampUp('f', 27.5);
  shootWhenReady(180, 500, true);
  setGlobalTargetAngle(-77);
  driveRampUp('f', 19.5);
  turnToTarget(-89, 100);
  driveRampUp('b', 25);
  turnToTarget(-177, 100);
  startIntakeOut();
  driveRampUp('b', 53);
  turnToTarget(-210, 100);
  startIntakeOut();
  driveRampUp('f', 32);
  startIntake();
  driveRampUp('f', 4.5);
}*/
void testAuto()
{

  startFlywheel(2475);
  for (int i = 0; i < 20; i++)
  {
    shootWhenReady(800, false);
    pros::delay(500);
  }

  // capScraper.tare_position();
  // setCapScraperTargetPos(-130);
  // startFlywheel(2600);
  // shootWhenReady(2550, 700, false);
  // driveShootAsync('f', 31, 12);

  //startIntake();
  //driveShootAsync('f', )
  // startFlywheel(2650);

  // startIntake();
  // pros::delay(2000);
  // stopIntake();
  // for (int i = 0; i < 20; i++)
  // {
  //   shootWhenReady(800, false);
  //   pros::delay(500);
  // }

  //moveCapScorer(480);
  //startFlywheel(2440);

  // startFlywheel(2650);
  // startIntake();
  // driveRampUp('f', 25);
  // driveRampUp('b', 5);
  // stopIntake();
  // turnToTarget(-54.25, 100);
  // shootWhenReady(700, true);

  //Red Cross Court Shooting//
  /*startFlywheel(188);
  startIntake();
  driveRampUp('f', 37);
  drive('b', 5);
  turnToTarget(-57.5, 100);
  pros::delay(1300); //Just to wait for the other alliance to shoot before countering
  shootWhenReady(600, false);
  startFlywheel(169);
  turnToTarget(-59, 100);
  pros::delay(700);
  shootWhenReady(500, true);
  stopIntake();
  turnToTarget(0, 100);
  drive('f', 4);
  turnToTarget(89, 100);
  moveCapScorer(500);
  drive('b', 32, 9000);*/
  //Red Cross Court Shooting ends here//

  /*startIntake();
  startFlywheel(180);
  shootWhenReady(600, false);
  std::cout << "Shot once\n";
  startFlywheel(150);
  pros::delay(700);
  shootWhenReady(600, true);*/

  /*startIntake();
  startFlywheel(180);
  shootWhenReady(600, false);
  startFlywheel(160);
  pros::delay(500);
  shootWhenReady(600, true);*/
  /*for (int i = 0; i < 20; i++)
  {
    pros::delay(1000);
    shootWhenReady(1000, false);
  }*/
  //turnToTarget(-67, 100);

  //moveCapScorer(270);

  /*startIntakeOut();
  driveRampUp('f', 33);
  startIntake();
  drive('f', 6);
  driveRampUp('b', 5);
  stopIntake();
  startFlywheel(190);
  driveRampUp('b', 34);
  turnToTarget(-89, 100);
  driveRampUp('f', 56.5);
  shootWhenReady(180, 500, false);
  driveRampUp('f', 20.5);
  shootWhenReady(180, 800, true);
  setGlobalTargetAngle(-99);
  driveRampUp('f', 18);
  setGlobalTargetAngle(-90);
  driveRampUp('b', 48);
  turnToTarget(0, 100);
  startIntakeOut();
  startFlywheel(180);
  driveRampUp('f', 35);
  startIntake();
  drive('f', 6);
  driveRampUp('b', 8);
  turnToTarget(-72.25, 100);
  driveRampUp('f', 27.5);
  shootWhenReady(180, 500, true);
  setGlobalTargetAngle(-77);
  driveRampUp('f', 19.5);
  turnToTarget(-89, 100);
  driveRampUp('b', 25);
  turnToTarget(-177, 100);
  driveRampUp('b', 53);
  driveRampUp('f', 5);
  turnToTarget(-280, 100);
  driveRampUp('f', 46.5);
  turnToTarget(-350, 100);
  driveRampUp('b', 83);*/
}
void autoOriginal() //Blue Front Original
{
}
void auto1() //Blue Front (Slot 1)
{
  startFlywheel(190);
  startIntake();
  driveRampUp('f', 35);
  drive('b', 5);
  driveRampUp('b', 30.25);
  turnToTarget(84, 100);
  drive('f', 10);
  shootWhenReady(180, 1000, false);
  drive('f', 19);
  shootWhenReady(180, 1000, true);
  setGlobalTargetAngle(101);
  drive('f', 18.5);
  driveRampUp('b', 26);
  turnToTarget(176, 100);
  drive('b', 6);
  moveCapScorer(480);
  stopIntake();
  turnToTarget(210.5, 100);
  drive('b', 47);
}

void auto2() //Blue Back (Slot 2)
{
  startFlywheel(177);
  startIntake();
  driveRampUp('f', 35);
  drive('b', 5);
  driveRampUp('b', 16);
  turnToTarget(65, 100);
  shootWhenReady(600, false);
  startFlywheel(164);
  pros::delay(700);
  shootWhenReady(500, true);

  stopIntake();
  turnToTarget(0, 100);
  driveRampUp('f', 14.5);
  turnToTarget(-89, 100);
  moveCapScorer(500);
  drive('b', 26.5, 8000);
}

void auto3() //Red Front (Slot 3)
{
  capScraper.tare_position();
  setCapScraperTargetPos(capScraper.get_position());
  startFlywheel(2650);
  driveRampUp('f', 25.5);
  startIntake();
  pros::delay(400);
  driveRampUp('b', 25.5);
  turnToTarget(-87.5, 100);
  stopIntake();
  setCapScraperTargetPos(-100);
  shootWhenReady(2600, 700, false);
  driveShootAsync('f', 31, 12);
  startFlywheel(2400);
  driveRampUp('b', 28);
  turnToTarget(-39.8, 100);
  driveRampUp('f', 15);
  startIntake();
  moveCapScorer(-240);
  driveRampUp('b', 14);
  setCapScraperTargetPos(-285);
  driveRampUp('f', 11);
  shootWhenReady(2300, 700, false);
  startFlywheel(1700);
  pros::delay(1000);
  shootWhenReady(1600, 1500, true, true);
  moveCapScorer(-100);
}

void auto4() //Red Back Middle Pole (Slot 4)
{
  capScraper.tare_position();
  setCapScraperTargetPos(capScraper.get_position());
  startFlywheel(2500);
  driveRampUp('f', 25);
  startIntake();
  pros::delay(500);
  driveRampUp('b', 12);
  setCapScraperTargetPos(-280);
  turnToTarget(52, 100);
  stopIntake();
  driveRampUp('f', 17);
  moveCapScorer(-100);
  driveRampUp('b', 40);
  turnToTarget(-48.5, 100);
  rapidFire(2460, 2000, true);
  startIntake();
  turnToTarget(0, 100);
  driveRampUp('f', 10);
  driveRampUp('f', 30);
}
void crossCourtRed() //Denoted with 20 (Slot 5)
{
  startFlywheel(186.5);
  startIntake();
  driveRampUp('f', 37);
  drive('b', 5);
  turnToTarget(-57.5, 100);
  pros::delay(1300); //Just to wait for the other alliance to shoot before countering
  shootWhenReady(600, false);
  startFlywheel(169);
  pros::delay(700);
  shootWhenReady(500, true);
  stopIntake();
  turnToTarget(0, 100);
  drive('f', 4);
  turnToTarget(89, 100);
  moveCapScorer(500);
  drive('b', 32, 9000);
}
void crossCourtBlue() //Denoted with 30 (Slot 6)
{
  startFlywheel(188);
  startIntake();
  driveRampUp('f', 37);
  drive('b', 5);
  turnToTarget(50.5, 100);
  pros::delay(1300); //Just to wait for the other alliance to shoot before countering
  shootWhenReady(600, false);
  startFlywheel(169);
  pros::delay(700);
  shootWhenReady(500, true);
  stopIntake();
  turnToTarget(0, 100);
  drive('f', 4);
  turnToTarget(-89, 100);
  moveCapScorer(500);
  drive('b', 32, 9000);
}
void programmingSkills() //Denoted with 10
{
  startIntakeOut();
  driveRampUp('f', 33);
  startIntake();
  drive('f', 6);
  driveRampUp('b', 5);
  startFlywheelVoltage(11000);
  driveRampUp('b', 34);
  turnToTarget(-89, 100);
  driveRampUp('f', 56.5);
  shootWhenReady(180, 500, false);
  driveRampUp('f', 20.5);
  shootWhenReady(180, 800, true); //Shoot bottom flag
  setGlobalTargetAngle(-99);
  driveRampUp('f', 18);
  setGlobalTargetAngle(-90);
  driveRampUp('b', 48);
  turnToTarget(-1, 100);
  startIntakeOut();
  startFlywheelVoltage(11000);
  driveRampUp('f', 35);
  startIntake(); //Getting ball from next cap
  drive('f', 6);
  driveRampUp('b', 8);
  turnToTarget(-72.25, 100);
  driveRampUp('f', 29);
  shootWhenReady(180, 500, true); //Shoot middle flag in middle pole
  setGlobalTargetAngle(-85);
  drive('f', 19.5);
  turnToTarget(-89, 100);
  driveRampUp('b', 23);
  //Turn to cap next to platform
  turnToTarget(-175, 100);
  driveRampUp('b', 34);
  moveCapScorer(480);
  turnToTarget(-315, 100);
  driveRampUp('f', 32);
  turnToTarget(-175, 100);
  startIntake();
  driveRampUp('f', 29);
  startFlywheelVoltage(11000);
  drive('f', 5);
  driveRampUp('b', 22);
  turnToTarget(-87, 100);
  drive('f', 8);
  shootWhenReady(180, 600, true);
  turnToTarget(52, 100);
  driveRampUp('f', 35);
  turnToTarget(-4, 100);
  drive('b', 10);
  drive('b', 30, 9000);
  drive('b', 25, 9000);
  //turnToTarget()
}

void autonomous()
{

  //pros::Task flywheelRPMMonitor(maintainFlywheelSpeed, parameter3, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Flywheel speed task");
  //pros::Task intakeMonitor(monitorIntake, parameter2, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Intake auto movement task");
  autoMode = 3;
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
  else if (autoMode == 10)
  {
    programmingSkills();
  }
  else if (autoMode == 20)
  {
    crossCourtRed();
  }
  else if (autoMode == 30)
  {
    crossCourtBlue();
  }
  else
  {
    testAuto();
  }
}
