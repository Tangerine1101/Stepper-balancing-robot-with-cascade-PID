#include <control.h>

Control::Control(int M1, int M2) {
    this->pin_MA = M1;
    this->pin_MB = M2;
    pinMode(M1, OUTPUT);
    pinMode(M2, OUTPUT);
    rpm = 0;
    angular = 0;
}
// Public functions
// example: motor1.initEncoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, TIM4);
void Control::initEncoder(int E1, int E2, TIM_TypeDef* timer) {
    this->pin_EA = E1;
    this->pin_EB = E2;
    //pinMode(E1, INPUT_PULLUP);
    //pinMode(E2, INPUT_PULLUP);
    this->TIM = timer;
    lastPulse = (uint16_t)TIM->CNT;
}

void Control::run(float voltage) {
    voltage = constrain(voltage, -MOTOR_MAX_VOLTAGE, MOTOR_MAX_VOLTAGE);

    if(voltage > 0){
        float pwm = this->map(voltage, 0, MOTOR_MAX_VOLTAGE, 0, MAX_PWM);
        analogWrite(pin_MA, pwm);
        analogWrite(pin_MB, 0);
    }
    else{
        float pwm = this->map(-voltage, 0, MOTOR_MAX_VOLTAGE, 0, MAX_PWM);
        analogWrite(pin_MA, 0);
        analogWrite(pin_MB, pwm);
    }
}

void Control::stop() {
    analogWrite(pin_MA, 0);
    analogWrite(pin_MB, 0);
}

void Control::update() {
    uint16_t currentPulse = (uint16_t)TIM->CNT;
    int16_t deltaPulse = (int16_t)(currentPulse - lastPulse);
    lastPulse = currentPulse;
    this->_speed = (float)deltaPulse / pulse_per_rev * 1000.0f / SAMPLE_TIME; //RPS -> 2pi Rad/s
    this->_angular += deltaPulse/pulse_per_rev;
    rpm = _speed * 60; //rpm
    angle = _angular * 360;
    rad_s = _speed * 2 * PI;
    angular = _angular * 2 * PI;

}

void Control::writeSpeed(float value) {
    this->_speed = value;
}

uint16_t Control::getEncoder() {
    return (uint16_t)TIM->CNT;
}

void Control::writeEncoder(uint16_t value) {
    TIM->CNT = value;
}

void Control::writeAngle(float value) {
    this->_angular = value;
}

float Control::getAngle() {
    return this->_angular * 2 * PI;
}
// Private functions
float Control::map(float x, float in_min, float in_max, float out_min, float out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//end