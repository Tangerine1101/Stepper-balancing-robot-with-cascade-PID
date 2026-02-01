#ifndef CONTROL_H
#define CONTROL_H
#include "config.h"

class Control {
    public:
        Control(int M1, int M2);
        void initEncoder(int E1, int E2, TIM_TypeDef* timer);
        uint16_t getEncoder();
        void update();
        void writeSpeed(float value);
        void writeEncoder(uint16_t value);
        void writeAngle(float value);
        float getAngle();
        void run(float voltage);
        void stop();
        void reset();
        volatile unsigned long* encoderValue;
        float rad_s, rpm, angle, angular;
    private:
        float map(float x, float in_min, float in_max, float out_min, float out_max);
        int pin_MA, pin_MB;
        int pin_EA, pin_EB;
        float _speed, _angular; // unit for speed & angular is RPM & 2pi*rad respectively
        TIM_TypeDef *TIM;
        uint16_t lastPulse =0;
        float lastAngle = 0;
};

#endif // 