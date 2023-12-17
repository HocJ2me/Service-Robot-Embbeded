#include "math_util.h"

int32_t Math_Constrain(int32_t value, int32_t min, int32_t max)
{
	if(value > max)value = max;
	else if(value < min)value = min;
	return value;
}

float Math_fConstrain(float value, float min, float max)
{
	if(value > max)value = max;
	else if(value < min)value = min;
	return value;
}

int16_t Math_abs(int16_t value)
{
	if((value > 0) || (value == 0))
		return value;
	return -value;
}

int16_t Math_min(int16_t value1, int16_t value2)
{
	if(value1 < value2)return value1;
	return value2;
}

int16_t Math_max(int16_t value1, int16_t value2)
{
	if(value1 > value2)return value1;
	return value2;
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}