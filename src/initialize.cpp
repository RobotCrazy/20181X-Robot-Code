#include "main.h"

#define CAP_FLIPPER 9
#define VISION_SENSOR_PORT 13

pros::Motor flipper(CAP_FLIPPER);

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

/*
int targetFlywheelSpeed = 0;
bool maintainFlywheelSpeedRequested = false;
bool flywheelOnTarget = false;
char *parameter3;
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

			if (abs(error) < 6)
			{
				flywheelOnTarget = true;
			}
			else
			{
				flywheelOnTarget = false;
			}
			finalAdjustment = error * kp; //add the rest of PID to this calculation
			flywheel.move_voltage(flywheel.get_voltage() + finalAdjustment);
			std::cout << targetFlywheelSpeed << "\n";
		}
		else
		{
			flywheel.move_voltage(0);
			flywheelOnTarget = false;
		}
		pros::delay(5);
	}
}*/

pros::Task flywheelRPMMonitor(maintainFlywheelSpeed, parameter3, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Flywheel speed task");
pros::Task intakeMonitor(monitorIntake, parameter2, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Intake auto movement task");

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

	flipper.tare_position();
	visionSensor.clear_led();
	flywheel.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_COAST);

	pros::Controller master(CONTROLLER_MASTER);
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
	autonSelector();
}
