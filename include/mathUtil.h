#ifndef mathUtil_h
#define mathUtil_h
#define PI atan(1) * 4

double degreeToRadian(double degrees)
{
  return degrees * (PI / 180.0);
}

bool isBetween(float number, float rangeLower, float rangeUpper)
{
  return (number > rangeLower && number < rangeUpper);
}
#endif