#ifndef __MAIN_H__
#define __MAIN_H__
#include "common_define.h"
#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

// #define DEBUG 0
// #define ENABLE_IMU 0
// #define ENABLE_SD 0
// #define ENABLE_HCSR 0

// #define HEART_BEAT_RATE 2
// #define IMU_PUBLISH_RATE 20     // hz
// #define COMMAND_RATE 40         // hz
// #define OPTICAL_PUBLISH_RATE 10 // hz
// #define DEBUG_RATE 5
// #define GET_BATTERY_RATE 0.33
// #define SD_LOGGING_RATE 25       // hz
// #define SONAR_LOOP_RATE 10          // hz

/* thongnd5 */
// #define TUNE_PID
#define ROS_FIRMWARE 1
#define DRIVER_SELF_INIT_TIME 6000 // ms
#define LS_MOVE_ACCEL_DECEL_VAL 400000

/*
    lino base config - Kinematic related configurations
*/
#ifndef LINO_BASE_CONFIG_H
#define LINO_BASE_CONFIG_H
// uncomment the base you're building
#define LINO_BASE DIFFERENTIAL_DRIVE // 2WD and Tracked robot w/ 2 motors
// #define LINO_BASE SKID_STEER         // 4WD robot
// #define LINO_BASE ACKERMANN          // Car-like steering robot w/ 2 motors
// #define LINO_BASE ACKERMANN1         // Car-like steering robot w/ 1 motor
// #define LINO_BASE MECANUM            // Mecanum drive robot

// uncomment the motor driver you're using
#define USE_L298_DRIVER
// #define USE_BTS7960_DRIVER
// #define USE_ESC

// uncomment the IMU you're using
//  #define USE_GY85_IMU
#define USE_MPU6050_IMU
// #define USE_MPU9150_IMU
// #define USE_MPU9250_IMU

#define K_P 1.15 // P constant
#define K_I 0.3  // I constant
#define K_D 0.01 // D constant

// define your robot' specs here
#define MAX_RPM 5000              // motor's maximum RPM
#define COUNTS_PER_REV 27000      // wheel encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.159      // wheel's diameter in meters
#define PWM_BITS 8                // PWM Resolution of the microcontroller
#define LR_WHEELS_DISTANCE 0.4118 // distance between left and right wheels
#define FR_WHEELS_DISTANCE 0.136  // distance between front and rear wheels. Ignore this if you're on 2WD/ACKERMANN
#define MAX_STEERING_ANGLE 0.415  // max steering angle. This only applies to Ackermann steering

// define Motor ID
#define MOTOR_LEFT_ID       3
#define MOTOR_RIGHT_ID      1

#define MOTOR_FRONT_ID      4
#define MOTOR_BACK_ID       2

#define PROMIXITY_SENSOR_LEFT_FRONT     X3
#define PROMIXITY_SENSOR_RIGHT_FRONT    X2

#define PROMIXITY_SENSOR_CHARGE_LEFT    X4
#define PROMIXITY_SENSOR_CHARGE_RIGHT   X5

#define PROXIMITY_SENSOR_LOW    X4
#define PROXIMITY_SENSOR_HIGH   X5

#define PROXIMITY_SENSOR_FRONT_LOW  X1
#define PROXIMITY_SENSOR_FRONT_HIGH X0

#define PROXIMITY_SENSOR_BACK_LOW   X3
#define PROXIMITY_SENSOR_BACK_HIGH  X2

#define LED_BUILTIN (13)
#define INPUT   0
#define OUTPUT  1

#define LIGHT_LED_ON digitalWriteFast(LIGHT_LED, LOW);
#define LIGHT_LED_OFF digitalWriteFast(LIGHT_LED, HIGH);

// MOTOR PINS
#ifdef USE_L298_DRIVER
#define MOTOR_DRIVER L298

#define MOTOR1_PWM 7

#define MOTOR1_IN_A 5
#define MOTOR1_IN_B 6

#define MOTOR2_PWM 2
#define MOTOR2_IN_A 4
#define MOTOR2_IN_B 3

#define MOTOR3_PWM 8
#define MOTOR3_IN_A 9
#define MOTOR3_IN_B 10

#define MOTOR4_PWM 13
#define MOTOR4_IN_A 12
#define MOTOR4_IN_B 11

#define PWM_MAX pow(2, PWM_BITS) - 1
#define PWM_MIN -PWM_MAX
#endif

#ifdef USE_BTS7960_DRIVER
#define MOTOR_DRIVER BTS7960

#define MOTOR1_PWM 1 // DON'T TOUCH THIS! This is just a placeholder
#define MOTOR1_IN_A 21
#define MOTOR1_IN_B 20

#define MOTOR2_PWM 8 // DON'T TOUCH THIS! This is just a placeholder
#define MOTOR2_IN_A 5
#define MOTOR2_IN_B 6

#define MOTOR3_PWM 0 // DON'T TOUCH THIS! This is just a placeholder
#define MOTOR3_IN_A 22
#define MOTOR3_IN_B 23

#define MOTOR4_PWM 2 // DON'T TOUCH THIS! This is just a placeholder
#define MOTOR4_IN_A 4
#define MOTOR4_IN_B 3

#define PWM_MAX pow(2, PWM_BITS) - 1
#define PWM_MIN -PWM_MAX
#endif

#ifdef USE_ESC
#define MOTOR_DRIVER ESC

#define MOTOR1_PWM 1 // DON'T TOUCH THIS! This is just a placeholder
#define MOTOR1_IN_A 21
#define MOTOR1_IN_B 20

#define MOTOR2_PWM 8 // DON'T TOUCH THIS! This is just a placeholder
#define MOTOR2_IN_A 5
#define MOTOR2_IN_B 6

#define MOTOR3_PWM 0 // DON'T TOUCH THIS! This is just a placeholder
#define MOTOR3_IN_A 22
#define MOTOR3_IN_B 23

#define MOTOR4_PWM 2 // DON'T TOUCH THIS! This is just a placeholder
#define MOTOR4_IN_A 4
#define MOTOR4_IN_B 3

#define PWM_MAX 400
#define PWM_MIN -PWM_MAX
#endif

#define STEERING_PIN 30

void publishOpticalSensor();
void publish_battery_state();
void stopBase();
void moveBase();
void moveLifting();
uint8_t handle_control_led_request(void);
uint8_t check_sonar(float sonar_d, const float stop_distance);
uint8_t check_sonar_safety(void);
void check_and_switch_system_state();
void run_in_state_normal();
void check_battery_status();
void reset_wheel_speed_val();
void check_confirm_button();

void lifting_controller_up();
void lifting_controller_down();
void check_status_robot_moving();
void handle_lifting_callback();
void handle_reinit_motor();
#endif

#endif
