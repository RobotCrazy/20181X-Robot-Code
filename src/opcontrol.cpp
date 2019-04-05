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
int targetcapScraperPos = 0;
bool flywheelRPMDropped = false;

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
	int capScraperTargetPos = 0;
	float flywheelSpeed = 0;

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
			intake.move_velocity(200);
			indexer.move_velocity(0);
			indexer.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD);
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
			capScraper.move_velocity(200);
		}
		else if (master.get_digital(DIGITAL_DOWN))
		{
			capScraper.move_velocity(-125);
		}
		else
		{
			capScraper.move_velocity(0);
			capScraper.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
		}

		pros::delay(2);
	}
}
