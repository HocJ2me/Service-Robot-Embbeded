#include "MotorRS485.h"

const int FN000 = 10000;  // SETMODE MODBUS:  2
const int FN003 = 10003;  // SETMODE DONG CO CHAY SPEED: 2
const int FN004 = 10004;  // Chieu quay
const int FN0B8 = 10184;  // THOI GIAN TANG TOC
const int FN0B9 = 10185;  // THOI GIAN GIAM TOC
const int FN010 = 10016;  // 1 ENABLE, 0 DISNABLE Motor

const int FN087 = 10135;
const int FN088 = 10136;
const int FN151 = 10337;

const int CN000 = 20000;  // Set Speed Motor

const int DN00 = 0;   // xem toc do hien tai
const int DN02 = 2;   // xem ty le tai hien tai
const int DN08 = 8;   // xem toc do set hien tai
const int DN36 = 36;  // xem ma loi he thong
const int DN37 = 37;  // xem ma loi lan khoi dong truoc

const int GEAR_RATIO = 27;

MotorRS485::MotorRS485(int id, int acc_start, int acc_stop, int delay_time)
  : _id{ id }, _acceleration_start{ acc_start }, _acceleration_stop{ acc_stop }, _delay_time(delay_time)
{}

bool MotorRS485::init() 
{
    delay(this->_delay_time);
    if (!ModbusRTUClient.holdingRegisterWrite(this->_id, FN000, 2)) {
        return false;
    }

    delay(this->_delay_time);
    if (!ModbusRTUClient.holdingRegisterWrite(this->_id, FN003, 2)) {
        return false;
    }

    delay(this->_delay_time);
    if (!ModbusRTUClient.holdingRegisterWrite(this->_id, FN010, 0)) {
        return false;
    }

    delay(this->_delay_time);
    if (!ModbusRTUClient.holdingRegisterWrite(this->_id, FN0B8, this->_acceleration_start)) {
        return false;
    }

    delay(this->_delay_time);
    if (!ModbusRTUClient.holdingRegisterWrite(this->_id, FN0B9, this->_acceleration_stop)) {
        return false;
    }

    delay(100);
    if (!ModbusRTUClient.holdingRegisterWrite(this->_id, FN010, 1)) {
        return false;
    }

    return true;
}

bool MotorRS485::setRPM(int rpm)
{
    delay(this->_delay_time);
    if (!ModbusRTUClient.holdingRegisterWrite(this->_id, CN000, rpm)) {
        return false;
    }
    return true;
}

int MotorRS485::getRPM() 
{
    delay(this->_delay_time);
    int rpm = ModbusRTUClient.holdingRegisterRead(this->_id, DN00);
    if (rpm == -1) return 0;
    if (rpm > 32768) {
        rpm = rpm - 65536;
    }
    return rpm;
}

int MotorRS485::getErrorCode()
{
    delay(this->_delay_time);
    int error_code = ModbusRTUClient.holdingRegisterRead(this->_id, DN36);
    return error_code;
}

int MotorRS485::getLastErrorCode()
{
    delay(this->_delay_time);
    int error_code = ModbusRTUClient.holdingRegisterRead(this->_id, DN37);
    return error_code;
}

int MotorRS485::getTorqueProportional()
{
    delay(this->_delay_time);
    int torque_proportional = ModbusRTUClient.holdingRegisterRead(this->_id, DN02);
    return torque_proportional;
}

int MotorRS485::getVelocitiesProportional()
{
    delay(this->_delay_time);
    int vel_proportional = ModbusRTUClient.holdingRegisterRead(this->_id, FN087);
    return vel_proportional;
}

int MotorRS485::getVelocityTimestep()
{
    delay(this->_delay_time);
    int time_step = ModbusRTUClient.holdingRegisterRead(this->_id, FN088);
    return time_step;
}

int MotorRS485::getCurrentProportional()
{
    delay(this->_delay_time);
    int current_proportional = ModbusRTUClient.holdingRegisterRead(this->_id, FN151);
    return current_proportional;
}