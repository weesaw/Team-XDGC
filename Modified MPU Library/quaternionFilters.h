#ifndef _QUATERNIONFILTERS_H_
#define _QUATERNIONFILTERS_H_

#include <Arduino.h>

void MadgwickQuaternionUpdate(float * q, float ax, float ay, float az, float gx, float gy,
                              float gz, float mx, float my, float mz,
                              float deltat);
void MahonyQuaternionUpdate(float * q, float * eInt, float ax, float ay, float az, float gx, float gy,
                            float gz, float mx, float my, float mz,
                            float deltat);
//const float * getQ1();
//const float * getQ2();

#endif // _QUATERNIONFILTERS_H_
