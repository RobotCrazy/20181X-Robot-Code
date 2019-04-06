#include "capScraper.h"
#include "main.h"

pros::Motor capScraper(CAP_SCRAPER_PORT);

int capScraperTargetPos;
bool holdCapScraperRequested = false;

//const int capScraperMaxPos = 20; //Maximum height for cap scraper
//const int capScraperMinPos = 200;

void holdCapScraperPos()
{
  int error = capScraperTargetPos - capScraper.get_position();
  int tolerance = 10;
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
