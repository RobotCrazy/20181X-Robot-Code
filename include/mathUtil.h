#ifndef _MATHUTIL_H_
#define _MATHUTIL_H_

#define PI atan(1) * 4

extern double degreeToRadian(double degrees);
extern bool isBetween(float number, float rangeLower, float rangeUpper);
extern int sign(int num);

#endif