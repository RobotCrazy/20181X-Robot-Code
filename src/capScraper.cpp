#include "capScraper.h"
#include "main.h"

pros::Motor capScraper(CAP_SCRAPER_PORT);

int capScraperTargetPos = 0;
bool holdCapScraperRequested = false;

//const int capScraperMaxPos = 20; //Maximum height for cap scraper
//const int capScraperMinPos = 200;

void setCapScraperTargetPos(int targetPos)
{
  capScraperTargetPos = targetPos;
}
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
  int tolerance = 10;
  float kp = 1;
  setCapScraperTargetPos(targetPos);
  while (abs(error) > tolerance)
  {
    error = targetPos - capScraper.get_position();
    capScraper.move_velocity(error * kp);
    std::cout << error << "\n";
    pros::delay(1);
  }
  capScraper.move_velocity(0);

  std::cout << "Done cap scoring";
}

char *param4;
void holdCapScraperPosInAuton(void *parameter)
{
  int error = capScraperTargetPos - capScraper.get_position();
  int kp = 2;
  while (true)
  {
    error = capScraperTargetPos - capScraper.get_position();
    if (abs(error) > 10)
    {
      capScraper.move_velocity(error * kp);
    }
    else
    {
      capScraper.move_velocity(0);
    }

    pros::delay(20);
  }
}
pros::Task capScraperAutonHold(holdCapScraperPosInAuton, param4, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Autonomous Scraper Hold Pos Task");
