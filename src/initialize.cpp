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
				intake.move_velocity(150);
				indexer.move_velocity(200);
			}
			else if (!(isBetween(intakeSonar.get_value(), 30, 80)))
			{
				intake.move_velocity(150);
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
				intake.move_velocity(150);
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
bool runFlywheelAtVoltageRequested = false;
int targetFlywheelVoltage = 0;
bool flywheelOnTarget = false;
char *parameter3;
float currentFlywheelVoltage = 0;

float determineFlywheelVoltage(float targetVelocity)
{
	float voltage = ((0.6676889314051 * targetVelocity) - 7.03063980643);

	if (voltage > 127)
	{
		voltage = 127;
	}
	else if (voltage < 0)
	{
		voltage = 0;
	}
	else
	{
		return voltage;
	}
}
void maintainFlywheelSpeed(void *param)
{

	//Constants//
	float kp = .3;
	float ki = 0;
	float kd = 0;

	//PID Variables Here//
	float currentVelocity = flywheel.get_actual_velocity();
	float lastVelocity1 = 0;
	float lastVelocity2 = 0;
	float lastVelocity3 = 0;
	float averageVelocity = 0;
	float error = targetFlywheelSpeed - currentVelocity;
	int lastError = 0;
	int totalError = 0;
	int integralActiveZone = 8;

	int onTargetCount = 0;
	float finalAdjustment = error * kp; //add the rest of PID to this calculation

	//Temp Variable//
	int deltaTime = 0;

	while (true)
	{
		if (maintainFlywheelSpeedRequested == true)
		{
			currentVelocity = flywheel.get_actual_velocity();
			averageVelocity = ((currentVelocity + currentVelocity + lastVelocity1 +
													lastVelocity2 + lastVelocity3) /
												 5);
			error = targetFlywheelSpeed - averageVelocity;

			if (error < -2 || error > 17)
			{
				currentFlywheelVoltage = determineFlywheelVoltage(targetFlywheelSpeed);
			}
			else
			{
				finalAdjustment = (error * kp) + (totalError * ki);
				currentFlywheelVoltage += finalAdjustment;
				if (abs(error) > integralActiveZone && error != 0)
				{
					totalError += error;
				}
				else
				{
					totalError = 0;
				}
			}
			lastVelocity3 = lastVelocity2;
			lastVelocity2 = lastVelocity1;
			lastVelocity1 = currentVelocity;

			if (currentFlywheelVoltage > 127)
			{
				currentFlywheelVoltage = 127;
			}
			else if (currentFlywheelVoltage < 0)
			{
				currentFlywheelVoltage = 0;
			}

			flywheel.move(currentFlywheelVoltage);

			if (abs(error) < 4)
			{
				onTargetCount++;
			}
			else
			{
				onTargetCount = 0;
				flywheelOnTarget = false;
			}
			if (onTargetCount >= 50)
			{
				flywheelOnTarget = true;
			}
			else
			{
				flywheelOnTarget = false;
			}

			if (deltaTime >= 100)
			{
				std::cout << "Avg:" << averageVelocity << "\n";
				deltaTime = 0;
			}
			else
			{
				deltaTime += 20;
			}

			/*if (deltaTime >= 100)
			{
				std::cout << " " << averageVelocity << "\n";
				std::cout << "Flywheel Voltage: " << currentFlywheelVoltage << "\n";
				deltaTime = 0;
			}
			else
			{
				deltaTime += 20;
			}*/

			/*currentVelocity = flywheel.get_actual_velocity();
			averageVelocity = ((currentVelocity + currentVelocity + lastVelocity1 +
													lastVelocity2 + lastVelocity3) /
												 5);

			error = targetFlywheelSpeed - averageVelocity;

			if (abs(error) < integralActiveZone && error != 0)
			{
				totalError += error;
				if (totalError > 10 / ki)
				{
					totalError = 10 / ki;
				}
			}
			else
			{
				totalError = 0;
			}
			//Try printing out error - lastError value to see how much effect the D term is having
			//The loop is running so fast that error and lastError might usually be equal so the D term isn't
			//having any effect

			finalAdjustment = ((error * kp) + (totalError * ki) + ((error - lastError) * kd)); //add the rest of PID to this calculation
			if (abs(error) > 4)
			{
				currentFlywheelVoltage += finalAdjustment;
			}

			if (currentFlywheelVoltage > 127)
			{
				currentFlywheelVoltage = 127;
			}
			else if (currentFlywheelVoltage < 0)
			{
				currentFlywheelVoltage = 0;
			}

			flywheel.move(currentFlywheelVoltage);

			if (abs(error) < 6)
			{
				onTargetCount++;
			}
			else
			{
				std::cout << onTargetCount << "\n";
				onTargetCount = 0;
				flywheelOnTarget = false;
			}
			if (onTargetCount >= 35)
			{
				flywheelOnTarget = true;
				std::cout << "True" << onTargetCount << "\n";
			}
			else
			{
				flywheelOnTarget = false;
			}
			if (deltaTime >= 100)
			{
				/*std::cout << " " << error << "\n";
				std::cout << "Flywheel Voltage: " << currentFlywheelVoltage << "\n";
			deltaTime = 0;
		}
		else
		{
			deltaTime += 20;
		}
		lastError = error;
		lastVelocity3 = lastVelocity2;
		lastVelocity2 = lastVelocity1;
		lastVelocity1 = currentVelocity;
		* /
	}
	else if (flywheelAutoVelControl == true)
	{
		//pros::motor_pid_s_t flywheelPID = pros::Motor::convert_pid(0, 4.75, .0001, 1.8);
		pros::motor_pid_s_t flywheelPID = pros::Motor::convert_pid(.0000001, .0000000000001, .000001, 25);
		flywheel.set_vel_pid(flywheelPID);
		flywheel.move_velocity(targetFlywheelSpeed);
		std::cout << flywheel.get_vel_pid().kp << "\n";
		error = targetFlywheelSpeed - flywheel.get_actual_velocity();
		if (deltaTime >= 100)
		{
			//std::cout << flywheel.get_actual_velocity() << "\n";
			deltaTime = 0;
		}
		else
		{
			deltaTime += 2;
		}
		/*if (error 
			{
				onTargetCount++;
			}
			else
			{
				onTargetCount = 0;
				flywheelOnTarget = false;
			}
			if (onTargetCount >= 175)
			{
				flywheelOnTarget = true;
			}*/
			//std::cout << flywheel.get_actual_velocity() << "\n";
		}
		else if (runFlywheelAtVoltageRequested == true)
		{
			flywheel.move_voltage(targetFlywheelVoltage);
		}
		else
		{
			flywheel.move_voltage(0);
			flywheelOnTarget = false;
		}
		pros::delay(20);
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
	pros::Controller master(CONTROLLER_MASTER);

	flipper.tare_position();
	visionSensor.clear_led();
	flywheel.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_COAST);
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
