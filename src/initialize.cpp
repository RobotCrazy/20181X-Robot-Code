#include "main.h"

#define FRONT_LEFT_PORT 4
#define BACK_LEFT_PORT 7
#define FRONT_RIGHT_PORT 6
#define BACK_RIGHT_PORT 5
#define INTAKE_PORT 12
#define FLY_WHEEL 14
#define CAP_FLIPPER 9
#define INDEXER_PORT 19
#define VISION_SENSOR_PORT 13
#define INDEXER_SONAR_PORT_PING 'A'
#define INDEXER_SONAR_PORT_ECHO 'B'
#define INTAKE_SONAR_PORT_PING 'C'
#define INTAKE_SONAR_PORT_ECHO 'D'
#define GYRO_PORT 'E'

pros::Motor frontLeft(FRONT_LEFT_PORT);
pros::Motor backLeft(BACK_LEFT_PORT);
pros::Motor frontRight(FRONT_RIGHT_PORT, true);
pros::Motor backRight(BACK_RIGHT_PORT, true);
pros::Motor intake(INTAKE_PORT);
pros::Motor flywheel(FLY_WHEEL);
pros::Motor flipper(CAP_FLIPPER);
pros::Motor indexer(INDEXER_PORT, true);
pros::ADIUltrasonic indexerSonar(INDEXER_SONAR_PORT_PING, INDEXER_SONAR_PORT_ECHO);
pros::ADIUltrasonic intakeSonar(INTAKE_SONAR_PORT_PING, INTAKE_SONAR_PORT_ECHO);
pros::ADIGyro gyro(GYRO_PORT);
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

bool intakeUpRequested = false; //boolean for state of intake request
bool intakeOutRequested = false;
bool prepareShotRequested = false;
char *parameter2;
int targetShootingTicks = 0;
bool shootBallRequested = false;
void monitorIntake(void *param)
{
	while (true)
	{
		if (intakeUpRequested == true)
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
			}
			else
			{
				indexer.move_velocity(0);
				intake.move_velocity(0);
				intake.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
				indexer.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
			}
		}
		else if (prepareShotRequested == true)
		{
			if (!(isBetween(indexerSonar.get_value(), 50, 80)))
			{
				intake.move_velocity(200);
				indexer.move_velocity(200);
			}
			else
			{
				intake.move_velocity(0);
				indexer.move_velocity(0);
				indexer.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
			}
		}
		else if (intakeOutRequested == true)
		{
			intake.move_velocity(-200);
			indexer.move_velocity(0);
			indexer.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
		}
		else if (shootBallRequested == true)
		{
			intake.move_relative(targetShootingTicks, 200);
		}
		else
		{
			intake.move_velocity(0);
			indexer.move_velocity(0);
			indexer.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
		}
		pros::delay(5);
	}
}

int targetFlywheelSpeed = 0;
bool maintainFlywheelSpeedRequested = false;
bool flywheelOnTarget = false;
char *parameter3;
float currentFlywheelVoltage = 0;
void maintainFlywheelSpeed(void *param)
{

	//Constants//
	float kp = .2;
	float ki = 0;
	float kd = 80;

	//P Variables Here//
	int currentSpeed = flywheel.get_actual_velocity();
	int error = targetFlywheelSpeed - currentSpeed;
	int lastError = 0;
	int onTargetCount = 0;
	float finalAdjustment = error * kp; //add the rest of PID to this calculation

	while (true)
	{
		if (maintainFlywheelSpeedRequested == true)
		{
			currentSpeed = flywheel.get_actual_velocity();
			error = targetFlywheelSpeed - currentSpeed;

			/*std::cout << "" << currentSpeed << "\n";
			std::cout << "" << error << "\n";*/

			if (error == 0)
			{
				lastError = 0;
			}
			finalAdjustment = error * kp + ((error - lastError) * kd); //add the rest of PID to this calculation
			//std::cout << "" << finalAdjustment << "\n";
			currentFlywheelVoltage = currentFlywheelVoltage + finalAdjustment;
			if (currentFlywheelVoltage > 12000)
			{
				currentFlywheelVoltage = 12000;
			}
			flywheel.move_voltage(currentFlywheelVoltage);

			std::cout << "Current" << currentFlywheelVoltage << "\n";
			std::cout << "Actual" << flywheel.get_actual_velocity() << "\n";
			if (abs(error) < 4)
			{
				onTargetCount++;
			}
			else
			{
				onTargetCount = 0;
				flywheelOnTarget = false;
			}
			if (onTargetCount >= 250)
			{
				flywheelOnTarget = true;
			}
			if (flywheelOnTarget == true)
			{
				std::cout << "True\n";
			}
			else
			{
				std::cout << "False\n";
			}

			lastError = error;
		}
		else
		{
			flywheel.move_voltage(0);
			flywheelOnTarget = false;
		}
		pros::delay(2);
	}
}

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
	pros::Controller master(CONTROLLER_MASTER);

	flipper.tare_position();
	visionSensor.clear_led();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled()
{
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
