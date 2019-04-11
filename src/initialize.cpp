#include "main.h"

#define VISION_SENSOR_PORT 13

pros::Vision visionSensor(VISION_SENSOR_PORT);

bool readyToExitAutoSelector = false;
void autonSelector();

void exitAutoSelector()
{
	readyToExitAutoSelector = true;
}

void decrementAutoMode()
{
	if (autoMode == 1)
	{
		autoMode = 6;
	}
	else
	{
		autoMode--;
	}
}
void incrementAutoMode()
{
	if (autoMode == 6)
	{
		autoMode = 1;
	}
	else
	{
		autoMode++;
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{

	frontRight.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	backRight.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	frontLeft.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	backLeft.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	intake.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	flywheel.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	capScraper.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);

	capScraper.tare_position();
	visionSensor.clear_led();
	accelerX.calibrate();
	flywheel.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_COAST);

	pros::Controller master(CONTROLLER_MASTER);

	//autonSelector();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled()
{
}

int autoMode = 1;
pros::Controller master(CONTROLLER_MASTER);

/*void autonSelector()
{
	pros::lcd::register_btn0_cb(decrementAutoMode);
	pros::lcd::register_btn1_cb(exitAutoSelector);
	pros::lcd::register_btn2_cb(incrementAutoMode);
	pros::lcd::set_text(0, "Select Autonomous:");

	while (readyToExitAutoSelector == false)
	{
		if (autoMode == 1)
		{
			pros::lcd::set_text(1, "Blue Front");
		}
		else if (autoMode == 2)
		{
			pros::lcd::set_text(1, "Blue Front - Shoot first");
		}
		else if (autoMode == 3)
		{
			pros::lcd::set_text(1, "Blue Back");
		}
		else if (autoMode == 4)
		{
			pros::lcd::set_text(1, "Red Front");
		}
		else if (autoMode == 5)
		{
			pros::lcd::set_text(1, "Red Front - Shoot first");
		}
		else if (autoMode == 6)
		{
			pros::lcd::set_text(1, "Red Back");
		}
		pros::delay(2);
	}
	pros::lcd::shutdown();
}*/

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize()
{
}
