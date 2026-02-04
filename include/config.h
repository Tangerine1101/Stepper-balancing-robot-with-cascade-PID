#ifndef CONFIG_H
#define CONFIG_H
#include <Arduino.h>
// Misc config
#define usbPort SerialUSB
#define SERIAL_BAUD 115200
#define SAMPLE_TIME 5 //ms
inline constexpr int serialSampleTime = 100/SAMPLE_TIME;
#define pi 3.14159265359
#define MICRO_STEP 32
// stepper config
 //8 pin bus: GND M0 M1 DIR1 DIR2 PUL1 PUl2 M2
inline constexpr float rad_to_step = MICRO_STEP*200/(2*pi);
inline constexpr float step_to_rad = 1/rad_to_step;
#define MAX_SPEED 5*rad_to_step // step/s
#define MAX_ACCEL 1.5 // rad/s^2
inline constexpr float MAX_SPEED_RAD = MAX_SPEED*step_to_rad;
inline constexpr float MAX_ACCEL_RAD = MAX_ACCEL*step_to_rad;
#define MIN_SPEED 16//pps
// stm32f401CC Pins out list
#define ONBOARD_LED PC13 //Onboard led
#define BUTTON PA0 //Onboard button
//DRV8825 pinout
#define M0 PA1
#define M1 PA2
#define DIR1 PA3
#define DIR2 PA4
#define PUL1 PA5 //T2C1
#define PUL2 PA6 //T3C1
#define M2 PA7
//PID control
//Inner pid
#define setpoint -0.005 // rad
#define Kp_ang 3.65 //  4.25|3.5
#define Ki_ang 5.0  //      |5.00
#define Kd_ang 0.2//      |0.01

//MPU6050 config
#define SDA PB9 //White wire
#define SCL PB8 //Purple wire
#define MPU_INT PB4
#define ANGULAR_CALIBRATE   0.0311
//Communication config 
#define Max_arguments   5
#define NODE_SENDBYTE 0xAA
#define NODE_STARTBYTE 0xFE
#define ComPort usbPort

//I2C config
#define MPU6050_ADDR 0x68 // I2C address of the MPU-6050
#define i2cFREQ 400000 // 400kHZ
inline constexpr float i2cPERIOD = 1/i2cFREQ * 1000000;

//Variable
#define INPUTS 1 //voltage
#define OUTPUTS 4 //[body angular, body angular velocity, wheel angular, wheel angular velocity]
inline volatile bool dataReady = false;
//Sensors
inline constexpr float K[4] = {0, 0 ,0 , 0};
//Control
#define trustPrediction 0.00f
#define MPU_ALPHA 0.95f
//Function
#endif