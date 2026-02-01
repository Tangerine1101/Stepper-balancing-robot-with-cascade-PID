#include "config.h"
#include "serialCom.h"
#include "IWatchdog.h"
#include "ControlSpeed.h"
#include "LQR.h"
#include <Arduino.h>
#include <bit>
// declare variables
unsigned long preMicros = 0;
uint32_t runTime = micros();
float wheelPitch, wheelPitchRate;
float last_voltage_output = 0;
float controlVal =0;
// define objects
serialCommunication tx(0xAA);
ControlSpeed stepper1(TIM2, PUL1, DIR1);
ControlSpeed stepper2(TIM3, PUL2, DIR2);
LQR lqr(K, &wheelPitch, &wheelPitchRate);

// declare functions 
bool ifspin();
void initCounter();
void scanI2C();
void callback();
void fallExp();
void setMicroStep(unsigned int _microStep);
void stepperExperiment();
float pid_compute();
void callbackPID();
//ISR functions
void mpuDataReady(){
  dataReady = true;
  //callback();
  //fallExp();
}
void stepper1_ISR(){
  stepper1.handleInterrupt();
}
void stepper2_ISR(){
  stepper2.handleInterrupt();
}

void setup() {
  //initialize hardware
  pinMode(ONBOARD_LED, OUTPUT);
  pinMode(MPU_INT, INPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  setMicroStep(MICRO_STEP);
  
  stepper1.begin();
  stepper2.begin();
  stepper1.attachInterrupt(stepper1_ISR);
  stepper2.attachInterrupt(stepper2_ISR);
  stepper1.setReverseDir(true);
  //initialize objects
  usbPort.begin(SERIAL_BAUD);
  lqr.intiI2C();

  //initialize operating 
  while (digitalRead(BUTTON) && !usbPort) delay(10);    
  usbPort.println("<start>");
  scanI2C();

  preMicros = millis();
  runTime = micros();
  IWatchdog.begin(serialSampleTime*1000*10);
  attachInterrupt(digitalPinToInterrupt(MPU_INT), mpuDataReady, RISING);
}
void loop() {
  static int logTime = 0;
  if (dataReady){
    dataReady = false;
    wheelPitch = step_to_rad*(stepper1.getPosition() + stepper2.getPosition())/2;
    wheelPitchRate = step_to_rad*(stepper1.getSpeed() + stepper2.getSpeed())/2;
    //callback();
    callbackPID();
    //stepperExperiment();
    //fallExp();
    
    if (logTime++ >= serialSampleTime){
    float txData[Max_arguments] = {lqr.angular, lqr.gyroRate, wheelPitch, wheelPitchRate, lqr.getRawAngle()};  
    tx.sendBinary(txData);
    digitalWrite(ONBOARD_LED, !digitalRead(ONBOARD_LED));
    //logTime = 0;
    }
    IWatchdog.reload();
  }
}
void callback() {
  lqr.getStates();
  float u = lqr.compute();
  u = u/M_wheel;
  // Tích phân Euler: v_new = v_old + a * T_sample
  // SAMPLE_TIME được define trong config.h (ms)
  controlVal += u * (SAMPLE_TIME / 1000.0f);

  // Chuyển sang steps/s
  float speedStep = controlVal * rad_to_step;

  // Nạp vận tốc vào bộ điều khiển
  stepper1.setSpeed(speedStep);
  stepper2.setSpeed(speedStep);

  if (abs(lqr.angular) > 0.7) { // ~40 độ
        stepper1.setSpeed(0);
        stepper2.setSpeed(0);
  }
}
//define functions
void callbackPID() {
  lqr.getStates();
  controlVal = -pid_compute();

  float speedStep = controlVal * rad_to_step;

  // Nạp vận tốc vào bộ điều khiển
  stepper1.setSpeed(speedStep);
  stepper2.setSpeed(speedStep);

  if (abs(lqr.angular) > 0.7) { // ~40 độ
        stepper1.setSpeed(0);
        stepper2.setSpeed(0);
  }
}
bool ifspin(){
  if (micros() - preMicros >= SAMPLE_TIME*1000){
    preMicros = micros();
    return true;
  }
  else{
    return false;
  }
}

void scanI2C(){
  byte error, address;
  int nDevices = 0;

  Serial.println("Scanning...");

  for (address = 1; address < 127; address++) {
    // SoftWire sử dụng cú pháp y hệt thư viện Wire chuẩn
    sensors.beginTransmission(address);
    error = sensors.endTransmission();

    if (error == 0) {
      Serial.print("Device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("Scan complete.\n");
  }
}

void fallExp(){
  static int logTime = 0;
  static bool fallFlag = false;
  lqr.getStates();

  if (abs(lqr.angular) <= 0.02) runTime = micros();
  else if(abs(lqr.angular) >= 0.1 && !fallFlag){
    fallFlag = true;
    usbPort.print("fall: ");
    usbPort.println((micros()- runTime)/1000.0);
    while(1);
  }

  if (logTime >= serialSampleTime) {
    float txData[Max_arguments] = {lqr.angular, lqr.gyroRate, wheelPitch, wheelPitchRate, micros() - runTime};
    tx.sendText(txData);
    logTime = 0;
  }
  logTime++;
  IWatchdog.reload();
}
void stepperExperiment(){
  lqr.getStates();
  controlVal = 40; //rad/s^2
  float accelStep = controlVal * rad_to_step;
}
void setMicroStep(unsigned int _microStep){
    _microStep = constrain(_microStep, 1, 32);
    unsigned int mode = std::__bit_width(_microStep) -1;
    if (mode == 5) mode = 7;
    digitalWrite(M0, mode & 0x01);
    digitalWrite(M1, mode & 0x02);
    digitalWrite(M2, mode & 0x04);
}

float pid_compute(){
  float angular = lqr.angular;
  float gyroRate = lqr.gyroRate;
  static float LPF = 0;
  static float alpha = 1.0; // trust 90% on new value
  static float integral = 0;
  static float derivative = 0;
  static float prevError = 0;
  const float dt = SAMPLE_TIME/1000.0;

  float error = setpoint - angular;
  float output = Kp * error + Ki * integral + Kd * derivative;
  integral += error *dt;
  integral = constrain(integral, -MAX_SPEED_RAD, MAX_SPEED_RAD);
  derivative = gyroRate; //physically gyro rate is derivative of angular
  prevError = error;
  output = output*alpha + (1-alpha)*LPF;
  LPF = output;
  if(abs(output) <= offset) return 0;
  output = constrain(output, -MAX_SPEED_RAD, MAX_SPEED_RAD);
  return output;
}