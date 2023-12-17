#ifndef __OPTICAL_SENSOR_H__
#define __OPTICAL_SENSOR_H__

#include <Arduino.h>

class OpticalSensor
{
  private:
    int _pin;

  public:
    OpticalSensor(int pin);
    ~OpticalSensor(){};

    /* 0 - LOW, 1 - HIGH */
    int getValue();
};

OpticalSensor::OpticalSensor(int pin) : _pin{ pin }
{
    pinMode(_pin, INPUT);
}

int OpticalSensor::getValue()
{
    return digitalRead(this->_pin);
}
#endif  //__OPTICAL_SENSOR_H__
