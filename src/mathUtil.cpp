#include "mathUtil.h"

double degreeToRadian(double degrees)
{
  return 1; //degrees * (PI / 180.0);
}

bool isBetween(float number, float rangeLower, float rangeUpper)
{
  return (number > rangeLower && number < rangeUpper);
}