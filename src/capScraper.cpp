#include "capScraper.h"
#include "main.h"

pros::Motor capScraper(CAP_SCRAPER_PORT);

int capScraperTargetPos;
bool holdCapScraperRequested = false;

//const int capScraperMaxPos = 20; //Maximum height for cap scraper
//const int capScraperMinPos = 200;

void holdCapScraperPos(int targetPos)
{
  int error = targetPos - capScraper.get_position();
  int tolerance = 30;
  float kp = 1;
  if (holdCapScraperRequested == true)
  {
    if (abs(error) > tolerance)
    {
      capScraper.move_velocity(error * kp);
    }
    else
    {
      capScraper.move_velocity(0);
    }
  }
}

void moveCapScorer(int targetPos)
{
  int error = targetPos - capScraper.get_position();
  int tolerance = 30;
  float kp = 1;
  while (abs(error) > tolerance)
  {
    capScraper.move_velocity(error * kp);
  }
  capScraper.move_velocity(0);
  capScraper.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD);

  std::cout << "Done cap scoring";
}
