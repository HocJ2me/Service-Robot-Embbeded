#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <Servo.h>

#include "ros.h"
#include "ros/time.h"
#include "lino_msgs/PID.h" //header file for pid server

#include "Motor.h"
#include "PID.h"
#include "lino_base_config.h"
#include "std_msgs/Float64.h"

// #define ENCODER_OPTIMIZE_INTERRUPTS // comment this out on Non-Teensy boards
#include "Encoder.h"

#define IMU_PUBLISH_RATE 20 //hz
#define COMMAND_RATE 20 //hz
#define DEBUG_RATE 5

Encoder motor_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV);
Controller motor_controller(Controller::MOTOR_DRIVER, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
// Encoder motor_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV);
// Controller motor_controller(Controller::MOTOR_DRIVER, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
// Encoder motor_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV);
// Controller motor_controller(Controller::MOTOR_DRIVER, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
// Encoder motor_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV);
// Controller motor_controller(Controller::MOTOR_DRIVER, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

PID motor_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

float g_req_rpm = 0;

unsigned long g_prev_command_time = 0;

//callback function prototypes
void commandCallback(const std_msgs::Float64& cmd_msg);
void PIDCallback(const lino_msgs::PID& pid);

ros::NodeHandle nh;

ros::Subscriber<std_msgs::Float64> rpm_sub("rpm_vel", commandCallback);
ros::Subscriber<lino_msgs::PID> pid_sub("pid", PIDCallback);

std_msgs::Float64 rpm;
ros::Publisher rpm_pub("rpm", &rpm);

void setup()
{
    nh.initNode();
    nh.getHardware()->setBaud(57600);
    nh.subscribe(pid_sub);
    nh.subscribe(rpm_sub);
    nh.advertise(rpm_pub);

    while (!nh.connected()) {
        nh.spinOnce();
    }
    nh.loginfo("LINOBASE CONNECTED");
    delay(1);
}

void loop()
{
    static unsigned long prev_control_time = 0;
    static unsigned long prev_debug_time = 0;

    if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE)) {
        moveBase();
        prev_control_time = millis();
    }

    if ((millis() - g_prev_command_time) >= 400) {
        stopBase();
    }

    if (DEBUG) {
        if ((millis() - prev_debug_time) >= (1000 / DEBUG_RATE)) {
            printDebug();
            prev_debug_time = millis();
        }
    }

    nh.spinOnce();
}

void PIDCallback(const lino_msgs::PID& pid)
{
    motor_pid.updateConstants(pid.p, pid.i, pid.d);
}

void commandCallback(const std_msgs::Float64& cmd_msg)
{
    g_req_rpm = cmd_msg.data;
    g_prev_command_time = millis();
}

void moveBase()
{
    int current_rpm = motor_encoder.getRPM();
    motor_controller.spin(motor_pid.compute(g_req_rpm, current_rpm));

    //pass velocities to publisher object
    rpm.data = current_rpm;

    //publish rpm
    rpm_pub.publish(&rpm);
}

void stopBase()
{
    g_req_rpm = 0;
}

void printDebug()
{
    char buffer[50];

    sprintf(buffer, "Encoder FrontLeft  M1: %ld", motor_encoder.read());
    nh.loginfo(buffer);
}
