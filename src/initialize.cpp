#include "main.h"

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
	pros::lcd::initialize();

	frontRight.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	backRight.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	frontLeft.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	backLeft.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	intake.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	flywheel.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	flipper.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
	pros::Controller master(CONTROLLER_MASTER);

	flipper.tare_position();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */

int autoMode = 1;
pros::Controller master(CONTROLLER_MASTER);

/*void autonSelector()
{

	master.print(0, 4, "Select Auton: ");exit
	pros::delay(2000);

	//Documentation for this library: https://docs.littlevgl.com
	lv_obj_t *autonSelectorParent = lv_obj_create(lv_scr_act(), NULL);
	lv_obj_set_size(autonSelectorParent, 400, 400);

	lv_obj_t *redFront = lv_btn_create(autonSelectorParent, NULL);
	lv_obj_align(redFront, NULL, LV_ALIGN_IN_TOP_LEFT, 10, 10);

	lv_obj_t *redBack = lv_btn_create(autonSelectorParent, NULL);
	lv_obj_align(redBack, NULL, LV_ALIGN_IN_TOP_RIGHT, 10, 10);

	lv_obj_t *blueFront = lv_btn_create(autonSelectorParent, NULL);
	lv_obj_align(blueFront, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 10, 10);

	lv_obj_t *blueBack = lv_btn_create(autonSelectorParent, NULL);
	lv_obj_align(blueBack, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, 10, 10);
}*/

void autonSelector()
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
}

void competition_initialize()
{
	autonSelector();
}
