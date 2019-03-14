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

/**************Define important variables here***************************/
bool holdFlipperRequested = false;
int targetFlipperPos = 0;
bool flywheelRPMDropped = false;

void holdFlipper(char *param)
{
	float kp = 5;
	if (holdFlipperRequested == true)
	{
		int error = targetFlipperPos - flipper.get_position();

		if (error > 4)
		{
			flipper.move(error * kp);
		}
		else
		{
			flipper.move(0);
			flipper.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD);
		}
	}
}

/*bool maintainFlywheelSpeedRequested = false;
bool flywheelOnTarget = false;
bool doingFirstShot = true;
int targetFlywheelSpeed = 0;

void detectFlywheelSpeedDrop()
{
	int currentSpeed = flywheel.get_actual_velocity();
	if (flywheelOnTarget == true && targetFlywheelSpeed - currentSpeed > 10)
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
}*/

int targetDriveBasePosL = 0;
int targetDriveBasePosR = 0;
bool driveBaseTargetSet = false;
void setTargetDriveBasePos()
{
	targetDriveBasePosR = (frontRight.get_position() + backRight.get_position()) / 2;
	targetDriveBasePosL = (frontLeft.get_position() + backLeft.get_position()) / 2;

	driveBaseTargetSet = true;
}

void opcontrol()
{
	/*intakeUpRequested = false;
	prepareShotRequested = false;
	maintainFlywheelSpeedRequested = false;*/
	flywheelRPMMonitor.suspend();
	intakeMonitor.suspend();
	pros::Controller master(CONTROLLER_MASTER);
	//pros::Task flywheelRPMMonitor(maintainFlywheelSpeed, parameter, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Flywheel speed task");

	int leftDrive = 0;
	int rightDrive = 0;
	bool firePrinted = false;
	int flipperTargetPos = 0;
	float flywheelSpeed = 0;
	flipper.move_absolute(0, 180);

	while (true)
	{
		//std::cout << "Sonar: " << intakeSonar.get_value() << "\n";
		leftDrive = master.get_analog(ANALOG_LEFT_Y);
		rightDrive = master.get_analog(ANALOG_RIGHT_Y);
		frontLeft.move(leftDrive);
		backLeft.move(leftDrive);
		frontRight.move(rightDrive);
		backRight.move(rightDrive);

		if (master.get_digital(DIGITAL_L2))
		{
			if (!(isBetween(indexerSonar.get_value(), 50, 80)))
			{
				intake.move_velocity(200);
				indexer.move_velocity(200);
			}
			else if (!(isBetween(intakeSonar.get_value(), 30, 80)))
			{
				intake.move_velocity(200);
				indexer.move_velocity(0);
				indexer.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
			}
			else
			{
				intake.move_velocity(0);
				indexer.move_velocity(0);
				intake.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
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

		if (master.get_digital(DIGITAL_X))
		{
			if (driveBaseTargetSet == false)
			{
				setTargetDriveBasePos();
			}
			holdDrivePos(targetDriveBasePosL, targetDriveBasePosR);
		}
		else
		{
			driveBaseTargetSet = false;
		}

		if (master.get_digital(DIGITAL_R1))
		{
			flywheelSpeed = 12000;
			flywheel.move_voltage(flywheelSpeed);
			std::cout << flywheel.get_actual_velocity() << "\n";
		}
		else
		{
			flywheel.move_voltage(0);
			flywheel.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_COAST);
			/*if (flywheelSpeed > 0)
			{
				flywheelSpeed -= 3;
			}
			else if (flywheelSpeed < 0)
			{
				flywheelSpeed += 3;
			}
			flywheel.move(flywheelSpeed);*/
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
		else
		{
			flipper.move_velocity(0);
			flipper.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD);
		}

		pros::delay(2);
	}
}
