#ifndef __COMMON_DEFINE__
#define __COMMON_DEFINE__

#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#define VMR_FIRMWARE 0
#define VOR_FIRMWARE 1
#define ROBOT_FIRMWARE VMR_FIRMWARE
#define LS10520_DRIVER
//#define LS20530_DRIVER
/*
    Peripherals
*/
#define ENABLE_SONAR        1
#define ENABLE_BATTERY      1
#define ENABLE_ROS_LOG      0
#define DEBUG_FIRM          0
#define EEPROM_TEST         0
#define ENABLE_IMU          0
#define ENABLE_SD           0
#define ENABLE_ENV_SENSOR   1
#define ENABLE_LIFT_MOTOR   0
#define USE_MOTOR           1

/*
    Serial2 <=> RS485: battery info and industrial speaker
*/
#define RX2 7
#define TX2 8
/* define MCU GPIO pins */
/*
    P4
*/
#define CS 10   // sonar
#define MOSI 11 // sonar
#define MISO 12 // sonar
/* P6 */
#define SCL1 16 // sonar
#define SDA1 17 // sonar
/* P12 */
#define CS2 36 // sonar
#define CS3 37 // sonar

#define TX6 24 // sonar
#define RX6 25 // sonar
/* P8 */
#define PWM3 6   // sonar
#define PWM4 9   // sonar
#define OUT1B 32 // sonar
#define A12 26   // sonar

/* P12 */
#define CTX3 31 // sonar
#define CRX3 30 // sonar
#define PWM8 33 // sonar

/* to Embedded Computer <-> second Serial port */
#define TX8 35
#define RX8 34

#define A5 19
#define A6 20 // ws2812 TX /* non-blocking WS2812 Serial */

/*
    External debug - preserved
*/
#define RX7 28 /* SBUS RX receiver. devo7 or mc6c */
#define TX7 29

#define GPIO_PIN_COUNT 3

/**
 * @brief: High voltage - digital inputs
 *
 * @hardware: reads 6 proximity sensors
 */
#define INPUT_HIGHVOLT_PIN_COUNT 8
#define X0 13 /* Start/Stop button*/
#define X1 14 /* Emergency Button */
#define X2 15 // Right
#define X3 18 // LeftmoveBase
#define X4 41
#define X5 23
#define X6 21 // Automatic charge right
#define X7 22 // Automatic charge left

/**
 * @brief: digital relays - 12/24VDL
 * @hardware: Currently it's used for controlling: Den chieu sang (12V)
 * @description: active LOW
 */
#define RELAY_PIN_COUNT 4
#define Y0 38
#define LIGHT_LED 27
#define Y2 39
#define Y3 40

/*
    PWM high voltage
*/
#define PWM_PIN_COUNT 4
#define PWM1 2
#define PWM2 5
#define PWM5 3
#define PWM6 4

/*
    Application define macros
*/
#if ROBOT_FIRMWARE == VMR_FIRMWARE
    #define SONAR_SENSORS_COUNT 2
    #define EMERGENCY_BTN       X6
    #define WS2812_IO       (uint8_t)20
    #define WS2812_COUNT    (uint8_t)72
#elif ROBOT_FIRMWARE == VOR_FIRMWARE
    #define SONAR_SENSORS_COUNT 5
    #define EMERGENCY_BTN X5
    #define WS2812_IO (uint8_t)29
    #define WS2812_COUNT (uint8_t)32
#endif

/*
    SOFTWARE
*/
/**
 * @brief Execution rates (in Hz)
 */
#define IMU_PUBLISH_RATE    40
#define SONAR_LOOP_RATE     10
#define COMMAND_RATE        40
#define SD_LOGGING_RATE     25
#define DEBUG_RATE          5
#define HEART_BEAT_RATE     10
#define GET_BATTERY_RATE    1
#define GET_RELAY_RATE      0.5
#define SONAR_CHECK_RATE    1000 / 60
#define LED_BLINK_TIME      100
// #define LED_RATE 18
#define LED_RATE                10
#define OPTICAL_PUBLISH_RATE    10 // hz
#define STATUS_PUBLISH_RATE      1
#define SONAR_LOOP_RATE         10 // hz
#define TOOL_RATE               1
#define IR_RATE                 20
#define SOUND_RATE              0.25
#define ENV_SENSOR_RATE         0.5 // environment sensor(humidity, temp. gas)

extern float sonar_distance[SONAR_SENSORS_COUNT];

#define SONAR0 sonar_distance[0]
#define SONAR1 sonar_distance[1]
#define SONAR2 sonar_distance[2] /* robot head */
#define SONAR3 sonar_distance[3]
#define SONAR4 sonar_distance[4]
#define SONAR5 sonar_distance[5]
#define SONAR6 sonar_distance[6]
#define SONAR7 sonar_distance[7]
#define SONAR_STOP_DISTANCE_FRONT 20
#define SONAR_STOP_DISTANCE_A 10
#define SONAR_STOP_DISTANCE_B 10
#define CHARGE_STOP_DISTANCE  2
#define SONAR_SAFETY_RECOVERY_TIME 3000 // Time since last stop detected until it's allowed to move
/*
    led effect
*/
typedef enum
{
    SOLID,         // 0
    BLINK_PATTERN, // 1
    BLINK_FAST,    // 2
    BLINK_SLOW,    // 3
    FADE_SLOW,     // 4
    FADE_FAST,     // 5
    WAVERING       // 6
} led_effect_t;


#define ENCODER_NOISE 5 
typedef enum{
    STOP,       //0
    FORWARD,    //1
    BACKWARD,   //2
    TURN_RIGHT, //3
    TURN_LEFT   //4
} robot_moving_t;

typedef enum{
    SAFE,
    LOCK_FORWARD,
    LOCK_BACKWARD,
    LOCK_ROTATE
} lock_moving_t;
#endif
