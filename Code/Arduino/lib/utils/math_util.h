#ifndef __MATH_UTIL_H__
#define __MATH_UTIL_H__

#include <Arduino.h>
float Math_fConstrain(float value, float min, float max);
int32_t Math_Constrain(int32_t value, int32_t min, int32_t max);
int16_t Math_abs(int16_t value);
int16_t Math_min(int16_t value1, int16_t value2);
int16_t Math_max(int16_t value1, int16_t value2);
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
typedef struct {
    int p,i,d;
} pid_para_t;
#endif