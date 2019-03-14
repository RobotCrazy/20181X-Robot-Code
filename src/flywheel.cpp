#include "flywheel.h"

pros::Motor flywheel(FLY_WHEEL);

/******************************Flywheel Status Global Variables*******************************/
int targetFlywheelSpeed = 0;
int targetFlywheelVoltage = 0;
bool maintainFlywheelSpeedRequested = false;
bool runFlywheelAtVoltageRequested = false;
bool autoVelControl = false;
bool flywheelOnTarget = false;

/*****************************Flywheel Velocity Control Variables*****************************/
float currentFlywheelVoltage = 0;

/******************************Flywheel Status Functions**************************************/
void setFlywheelVoltage(int voltage)
{
  targetFlywheelVoltage = voltage;
}
void startFlywheelVoltage(int voltage)
{
  maintainFlywheelSpeedRequested = false;
  flywheelAutoVelControl = false;
  runFlywheelAtVoltageRequested = true;
  setFlywheelVoltage(voltage);
}
void startFlywheel(int targetSpeed)
{
  maintainFlywheelSpeedRequested = true;
  flywheelAutoVelControl = false;
  targetFlywheelSpeed = targetSpeed;
}
void startFlywheelAutoVelControl(int targetSpeed)
{
  maintainFlywheelSpeedRequested = false;
  flywheelAutoVelControl = true;
  targetFlywheelSpeed = targetSpeed;
}
void startFlywheel(int voltage, int targetSpeed)
{
  maintainFlywheelSpeedRequested = true;
  targetFlywheelSpeed = targetSpeed;
  flywheel.move_voltage(voltage);
}

void stopFlywheel()
{
  maintainFlywheelSpeedRequested = false;
  flywheelAutoVelControl = false;
  runFlywheelAtVoltageRequested = false;
  targetFlywheelSpeed = 0;
  targetFlywheelVoltage = 0;
}

/************************************Flywheel Velocity Control Functions*************************/
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

/******************************Flywheel Status Handling Task**********************************/
char *parameter3;
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