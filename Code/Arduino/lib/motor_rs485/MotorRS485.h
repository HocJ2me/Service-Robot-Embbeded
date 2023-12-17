#ifndef __MOTOR_RS485_H__
#define __MOTOR_RS485_H__

#include <ArduinoModbus.h>

class MotorRS485
{
public:
    MotorRS485(int id, int acc_up=0, int acc_down=0, int delay_time=5);
    ~MotorRS485() {}
    bool init();
    bool setRPM(int rpm);
    int  getRPM();
    int  getErrorCode();
    int  getLastErrorCode();
    int  getTorqueProportional();
    int  getVelocitiesProportional();
    int  getVelocityTimestep();
    int  getCurrentProportional();
private:
    int _id;
    int _acceleration_start; /* ms */
    int _acceleration_stop;  /* ms */
    int _delay_time; /* ms: delay time when send command  */
};

#endif //__MOTOR_RS485_H__
