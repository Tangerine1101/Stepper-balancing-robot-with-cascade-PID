#LQR balancing robot report:

## Theoretical background for Hardware
### MCU stm32f103C8
#### Timer:
##### Theorems
stm32f103C8 has 4 16bits timers, each timer has 4 channels. 
##### Timer's macros:
Macro | Description
------|------------
RCC|
APB1ENR|
RCC_APB1ENR_TIM3EN|
TIM3|
SMCR| set up counting mode via smccr register: \ 0b001: count on TI2 edge \ 010: count on TI1 edge \ 011: count on both TI1 and TI2 edges

### Mpu6050
#### Theorems
mpu6050 is a 6-axis motion tracking device that combines a 3-axis gyroscope and a 3-axis accelerometer. It has an on-board Digital Motion Processor (DMP) that processes the raw data from the gyroscope and accelerometer to provide accurate and reliable motion tracking data.

#### Registers
Register | Description
---------|------------
WHO_AM_I | 0x75
PWR_MGMT_1 | 0x6B
SMPLRT_DIV | 0x19
CONFIG | 0x1A
GYRO_CONFIG | 0x1B
ACCEL_CONFIG | 0x1C
ACCEL_XOUT_H | 0x3B
ACCEL_XOUT_L | 0x3C
ACCEL_YOUT_H | 0x3D
ACCEL_YOUT_L | 0x3E
ACCEL_ZOUT_H | 0x3F
ACCEL_ZOUT_L | 0x40
TEMP_OUT_H | 0x41
TEMP_OUT_L | 0x42
GY

###Stepper accel control
#### timer used:
TIM2, TIM3
## Prototype
### connect mapping:
#### DRV8825:
description | pin
----|------------
M0|PA1
M1|PA2
DIR1|PA3
DIR2|PA4
PUL1|PA5
PUL2|PA6
M2|PA7

#### Mpu6050:
pin | description
----|------------
SCL | PB8
SDA | PB9
INT | PB4
VCC | 3.3V
GND | GND

## Programming:
### workflow:
setup()->wait for button pressed or serial connect->scanI2C()->loop()->mpu interrupt trigger->callback()
callback() do:
- get mpu data
- calculate lqr
- call stepper control library's function and set stepper acceleration
- print data to serial every 'serialSampleTime'

## Modeling & identification:
### Model params:
parts|weight
-----|------
battery|66x4g
motor|0.21x2kg
wheel+connector|(32+14)x2g
misc|474g
total|1.250kg


K = -784.45,-70.4,-1.55,-4.06

##experiments:
###stepper max speed/accel
micro stepping| speed(rad/s)|acceleration(rad/s^2)
---|---|---
32|5.5|0.005
16|9.8|0.02
8|18.5|0.02
4|>55|0.02