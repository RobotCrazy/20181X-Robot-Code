#include "api.h"

#ifndef _CAPSCRAPER_H_
#define _CAPSCRAPER_H_

#define CAP_SCRAPER_PORT 13

extern pros::Motor capScraper;

extern int capScraperTargetPos;
extern bool holdCapScraperRequested;

//extern const int capScraperMaxPos = 20; //Maximum height for cap scraper
//extern const int capScraperMinPos = 200;

extern void holdCapScraperPos();

#endif