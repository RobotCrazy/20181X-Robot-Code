#include "mathUtil.h"
#include <cmath>

double degreeToRadian(double degrees)
{
  return degrees * (PI / 180.0);
}

bool isBetween(float number, float rangeLower, float rangeUpper)
{
  return (number > rangeLower && number < rangeUpper);
}