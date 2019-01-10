#include "main.h"

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

#define FRONT_LEFT_PORT 4
#define BACK_LEFT_PORT 7
#define FRONT_RIGHT_PORT 6
#define BACK_RIGHT_PORT 5
#define INTAKE_PORT 12
#define FLY_WHEEL 14
#define CAP_FLIPPER 9
#define INDEXER_PORT 19
#define BALL_SONAR_PORT_PING 'A'
#define BALL_SONAR_PORT_ECHO 'B'
#define GYRO_PORT 'D'

pros::Motor frontLeft(FRONT_LEFT_PORT);
pros::Motor backLeft(BACK_LEFT_PORT);
pros::Motor frontRight(FRONT_RIGHT_PORT, true);
pros::Motor backRight(BACK_RIGHT_PORT, true);
pros::Motor intake(INTAKE_PORT);
pros::Motor flywheel(FLY_WHEEL);
pros::Motor flipper(CAP_FLIPPER);
pros::Motor indexer(INDEXER_PORT, true);
pros::ADIUltrasonic ballSonar(BALL_SONAR_PORT_PING, BALL_SONAR_PORT_ECHO);
pros::ADIGyro gyro(GYRO_PORT);

/**************Define important variables here***************************/
bool holdFlipperRequested = false;
bool flywheelRPMDropped = false;

void holdFlipper(int pos)
{
	if (holdFlipperRequested == true)
	{
		int error = flipper.get_position() - pos;

		if (error > 10)
		{
			flipper.move_relative(error, 200);
		}
		else
		{
			flipper.set_brake_mode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
		}
	}
}

bool maintainFlywheelSpeedRequested = false;
bool flywheelOnTarget = false;
bool doingFirstShot = true;
int targetFlywheelSpeed = 0;
void detectFlywheelSpeedDrop()
{
	int currentSpeed = flywheel.get_actual_velocity();
	if (flywheelOnTarget == true && currentSpeed - targetFlywheelSpeed > 10)
	{
		targetFlywheelSpeed = 150;
		doingFirstShot = false;
		flywheelOnTarget = false;
	}
}

char *parameter;
void maintainFlywheelSpeed(void *param)
{
	float kp = 80;
	float ki = 0;
	float kd = 0;
	int currentSpeed = flywheel.get_actual_velocity();
	int error = targetFlywheelSpeed - currentSpeed;
	float finalAdjustment = error * kp; //add the rest of PID to this calculation

	while (true)
	{
		if (maintainFlywheelSpeedRequested == true)
		{
			currentSpeed = flywheel.get_actual_velocity();
			error = targetFlywheelSpeed - currentSpeed;

			if (flywheelOnTarget == false && abs(error) < 10)
			{
				flywheelOnTarget = true;
			}
			finalAdjustment = error * kp; //add the rest of PID to this calculation
			flywheel.move_voltage(flywheel.get_voltage() + finalAdjustment);
			std::cout << targetFlywheelSpeed << "\n";
			pros::delay(5);
		}
		else
		{
			currentSpeed = flywheel.get_actual_velocity();
			error = targetFlywheelSpeed - currentSpeed;
			finalAdjustment = currentSpeed * kp; //add the rest of PID to this calculation
			flywheel.move_voltage(0);
			flywheelOnTarget = false;
		}
		detectFlywheelSpeedDrop();
	}
}

void opcontrol()
{
	pros::Controller master(CONTROLLER_MASTER);
	pros::Task flywheelRPMMonitor(maintainFlywheelSpeed, parameter, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Flywheel speed task");

	int leftDrive = 0;
	int rightDrive = 0;
	bool firePrinted = false;
	int flipperTargetPos = 0;
	float flywheelSpeed = 0;

	while (true)
	{
		leftDrive = master.get_analog(ANALOG_LEFT_Y);
		rightDrive = master.get_analog(ANALOG_RIGHT_Y);
		frontLeft.move(leftDrive);
		backLeft.move(leftDrive);
		frontRight.move(rightDrive);
		backRight.move(rightDrive);

		if (master.get_digital(DIGITAL_L2))
		{
			if (!(isBetween(ballSonar.get_value(), 50, 80)))
			{
				intake.move_velocity(200);
				indexer.move_velocity(200);
			}
			else
			{
				intake.move_velocity(200);
				indexer.move_velocity(0);
				indexer.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
			}
		}
		else if (master.get_digital(DIGITAL_L1))
		{
			intake.move_velocity(200);
			indexer.move_velocity(200);
		}
		else if (master.get_digital(DIGITAL_R2))
		{
			intake.move_velocity(-200);
		}
		else if (master.get_digital(DIGITAL_A))
		{
			indexer.move_velocity(-200);
		}
		else
		{
			intake.move_velocity(0);
			indexer.move_velocity(0);
		}

		if (master.get_digital(DIGITAL_R1))
		{
			if (doingFirstShot == true)
			{
				targetFlywheelSpeed = 190;
				maintainFlywheelSpeedRequested = true;
			}
		}
		else
		{
			maintainFlywheelSpeedRequested = false;
			targetFlywheelSpeed = 0;
			doingFirstShot = true;
		}

		if (flywheel.get_actual_velocity() <= 164 && firePrinted == false)
		{
			master.print(0, 0, "Fire");
			firePrinted = true;
		}
		else if (flywheel.get_actual_velocity() < 150)
		{
			master.clear_line(0);
			firePrinted = false;
		}

		if (master.get_digital(DIGITAL_UP))
		{
			flipper.move_velocity(200);
			holdFlipperRequested = false;
		}
		else if (master.get_digital(DIGITAL_DOWN))
		{
			flipper.move_velocity(-200);
			holdFlipperRequested = false;
		}
		else if (master.get_digital(DIGITAL_RIGHT))
		{
			holdFlipperRequested = true;
			flipperTargetPos = 200;
		}
		else if (holdFlipperRequested == false)
		{
			flipper.move_velocity(0);
		}

		holdFlipper(flipperTargetPos);

		pros::delay(2);
	}
}
