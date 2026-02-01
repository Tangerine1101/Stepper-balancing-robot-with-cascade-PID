#ifndef CONFIG_H
#define CONFIG_H
#include <Arduino.h>
// Misc config
#define usbPort SerialUSB
#define SERIAL_BAUD 115200
#define SAMPLE_TIME 5 //ms
inline constexpr int serialSampleTime = 10/SAMPLE_TIME;
#define pi 3.14159265359
#define RADIAN_MODE 1
inline constexpr float processDelay = 0.005f; //total processing time in seconds
inline const int MICRO_STEP = 32;
// stepper config
 //8 pin bus: GND M0 M1 DIR1 DIR2 PUL1 PUl2 M2
 
// stm32f401CC Pins out list
#define ONBOARD_LED PC13 //Onboard led
#define BUTTON PA0 //Onboard button
//DRV8825 pinout
#define M0 PA1
#define M1 PA2
#define DIR1 PA4
#define DIR2 PA3
#define PUL1 PA5 //:T2C1
#define PUL2 PA6 //T3C1
#define M2 PA7


#define SDA PB9 //White wire
#define SCL PB8 //Purple wire
#define MPU_INT PB4
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
#define MOTOR_DIFF_CONST    0.997 // different of motor1 comparing to motor2
inline volatile bool dataReady = false;
#define VOLTAGE_COMPENSATION_CONST  1.0f
inline constexpr float K[4] = {25.67, 0.931, 0.0054, 1.0820};
//Control

#define MPU_ALPHA 0.05f
#define PREDICTION_HORIZON 0.0f
#define MOTOR_LAG 0.005f
#define Q_angle 0.005f // Nhiễu quá trình (tin vào mô hình)
#define Q_gyro 0.003f  // Nhiễu quá trình (tin vào gyro)
#define R_angle 0.03f  // Nhiễu đo lường (tin vào accel)
//Function




//wait for eliminated, used for control DC motor:

//Encoder config
#define ENCODER_PULSE_PER_REV 11
#define ENCODER_X_MODE  4
#define GEAR_RATIO 45
inline constexpr float pulse_per_rev = ENCODER_PULSE_PER_REV * ENCODER_X_MODE * GEAR_RATIO;
#define MAX_VOLTAGE 12.0f 
#define MIN_START_VOLTAGE 0.0f //friction compensation Voltage
inline constexpr float preProcessingVoltage = MAX_VOLTAGE - MIN_START_VOLTAGE; // pre-compensation voltage

#define MOTOR1_A PA3 //T2C3
#define MOTOR1_B PA2 //T2C4
#define MOTOR2_A PA1 //T2C2
#define MOTOR2_B PA5 //T2C1

#define MOTOR1_ENCODER_A PA6 //T3C1
#define MOTOR1_ENCODER_B PA7 //T3C2
#define MOTOR2_ENCODER_A PB6 //T4C1
#define MOTOR2_ENCODER_B PB7 //T4C2

//Timers 
#define MOTOR1_PWM TIM2
#define MOTOR2_PWM TIM2
#define MOTOR1_COUNTER TIM3
#define MOTOR2_COUNTER TIM4


//motor config 
#define PWM_FREQ 10000 // 
#define PWM_RES  10    // 12-bit resolution (0-4095)
#define MAX_PWM 4095
#define MOTOR_MAX_VOLTAGE MAX_VOLTAGE
#define MOTOR_MIN_VOLTAGE MIN_START_VOLTAGE

#endif

void initCounter() {
  // 1. Enable Clock cho GPIOA và GPIOB (nơi chứa chân Encoder)
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

  // 2. Cấu hình Pin PA6, PA7 (TIM3) sang chế độ Alternate Function (AF)
  // PA6 (TIM3_CH1)
  GPIOA->MODER  &= ~(3U << (6 * 2));    // Clear mode cũ
  GPIOA->MODER  |=  (2U << (6 * 2));    // Set mode AF (Binary 10)
  GPIOA->PUPDR  &= ~(3U << (6 * 2));    // Clear pull-up/down
  GPIOA->PUPDR  |=  (1U << (6 * 2));    // Set Pull-up (cần thiết cho encoder open-drain)
  GPIOA->AFR[0] &= ~(0xF << (6 * 4));   // Clear AF cũ
  GPIOA->AFR[0] |=  (2U << (6 * 4));    // Set AF2 (0010) cho TIM3

  // PA7 (TIM3_CH2)
  GPIOA->MODER  &= ~(3U << (7 * 2));
  GPIOA->MODER  |=  (2U << (7 * 2));
  GPIOA->PUPDR  &= ~(3U << (7 * 2));
  GPIOA->PUPDR  |=  (1U << (7 * 2));
  GPIOA->AFR[0] &= ~(0xF << (7 * 4));
  GPIOA->AFR[0] |=  (2U << (7 * 4));    // Set AF2

  // 3. Cấu hình Pin PB6, PB7 (TIM4) sang chế độ AF
  // PB6 (TIM4_CH1)
  GPIOB->MODER  &= ~(3U << (6 * 2));
  GPIOB->MODER  |=  (2U << (6 * 2));
  GPIOB->PUPDR  &= ~(3U << (6 * 2));
  GPIOB->PUPDR  |=  (1U << (6 * 2));
  GPIOB->AFR[0] &= ~(0xF << (6 * 4));
  GPIOB->AFR[0] |=  (2U << (6 * 4));    // Set AF2

  // PB7 (TIM4_CH2)
  GPIOB->MODER  &= ~(3U << (7 * 2));
  GPIOB->MODER  |=  (2U << (7 * 2));
  GPIOB->PUPDR  &= ~(3U << (7 * 2));
  GPIOB->PUPDR  |=  (1U << (7 * 2));
  GPIOB->AFR[0] &= ~(0xF << (7 * 4));
  GPIOB->AFR[0] |=  (2U << (7 * 4));    // Set AF2

  // 4. Cấu hình TIM3
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  TIM3->SMCR = 3;          // Encoder Mode 3 (đếm cả 2 cạnh TI1, TI2)
  TIM3->CCMR1 = 0;         // Reset CCMR1
  TIM3->CCMR1 |= (0xF << 4) | (0xF << 12); // Filter max (IC1F, IC2F) chống nhiễu
  TIM3->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P); // Polarity: Rising edge (mặc định)
  TIM3->ARR = 0xFFFF;      // Auto-reload max
  TIM3->CNT = 0;           // Reset counter
  TIM3->CR1 = TIM_CR1_CEN; // Enable Timer

  // 5. Cấu hình TIM4
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
  TIM4->SMCR = 3;
  TIM4->CCMR1 = 0; 
  TIM4->CCMR1 |= (0xF << 4) | (0xF << 12);
  TIM4->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);
  TIM4->ARR = 0xFFFF;
  TIM4->CNT = 0;
  TIM4->CR1 = TIM_CR1_CEN;
}