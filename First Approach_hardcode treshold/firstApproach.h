#ifndef _FIRSTAPPROACH_H_
#define _FIRSTAPPROACH_H_

#include <Arduino.h>

void check_ax1(float max, float min, uint8_t * output);
void check_ax2(float max, float min, uint8_t * output);
void check_ay1(float max, float min, uint8_t * output);
void check_ay2(float max, float min, uint8_t * output);
void check_az1(float max, float min, uint8_t * output);
void check_az2(float max, float min, uint8_t * output);
void check_gx1(float max, float min, uint8_t * output);
void check_gx2(float max, float min, uint8_t * output);
void check_gy1(float max, float min, uint8_t * output);
void check_gy2(float max, float min, uint8_t * output);
void check_gz1(float max, float min, uint8_t * output);
void check_gz2(float max, float min, uint8_t * output);

float getMax(float * data, int length);
float getMin(float * data, int length);

#endif // _FIRSTAPPROACH_H_