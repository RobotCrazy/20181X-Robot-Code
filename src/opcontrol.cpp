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
	//flywheelRPMMonitor.suspend();
	intakeMonitor.suspend();
	pros::Controller master(CONTROLLER_MASTER);
	//pros::Task flywheelRPMMonitor(maintainFlywheelSpeed, parameter, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Flywheel speed task");

	int leftDrive = 0;
	int rightDrive = 0;
	bool firePrinted = false;
	int capScraperTargetPos = 0;
	float flywheelSpeed = 0;

	bool firstShotDetection = false;

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
			indexer.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
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
			maintainFlywheelSpeedRequested = true;
			targetFlywheelSpeed = 2475;
			if (flywheelShotDetected == true)
			{
				targetFlywheelSpeed = 1750;
				if (firstShotDetection == false)
				{
					firstShotDetection = true;
					indexer.move_velocity(0);
					pros::delay(1000);
				}
			}
			//flywheel.move_voltage(12000);
			//std::cout << flywheel.get_actual_velocity() << "\n";
		}
		else
		{
			flywheel.move_voltage(0);
			flywheel.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_COAST);
			flywheelOnTarget = false;
			flywheelShotDetected = false;
			targetFlywheelSpeed = 0;
			maintainFlywheelSpeedRequested = false;
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
			capScraperTargetPos = capScraper.get_position();
			holdCapScraperRequested = false;
		}
		else if (master.get_digital(DIGITAL_DOWN))
		{
			capScraper.move_velocity(-125);
			capScraperTargetPos = capScraper.get_position();
			holdCapScraperRequested = false;
		}
		else
		{
			holdCapScraperRequested = true;
			holdCapScraperPos();
		}
		//std::cout << "onTarget" << flywheelOnTarget << "\n";
		//std::cout << flywheelShotDetected << "\n";
		pros::delay(20);
	}
}
