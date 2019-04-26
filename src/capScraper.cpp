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
  int holdingPower = 10;
  int error = targetPos - capScraper.get_position();
  int tolerance = 4;
  float kp = 1;
  if (holdCapScraperRequested == true)
  {
    if (abs(error) > tolerance)
    {
      capScraper.move(error * kp);
    }
    else
    {
      capScraper.move(holdingPower);
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
  int holdingPower = 10;
  int error = capScraperTargetPos - capScraper.get_position();
  int kp = 1;
  while (true)
  {
    error = capScraperTargetPos - capScraper.get_position();
    if (abs(error) > 3)
    {
      capScraper.move(error * kp);
    }
    else
    {
      capScraper.move(holdingPower);
    }
    std::cout << capScraper.get_position() << "\n";
    pros::delay(20);
  }
}
pros::Task capScraperAutonHold(holdCapScraperPosInAuton, param4, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Autonomous Scraper Hold Pos Task");
