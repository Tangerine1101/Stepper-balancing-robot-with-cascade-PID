#ifndef LQR_H
#define LQR_H
#include <Arduino.h>
#include <config.h>
#include <Wire.h>
#define INPUTS 1 //voltage
#define OUTPUTS 4 //[body angular, body angular velocity, wheel angular, wheel angular velocity]
#define sensors Wire

class LQR {
    public:
        LQR(const float K[OUTPUTS], float *wheelAngle, float *wheelSpeed);
        float compute();
        void intiI2C();
        void setK(float K[OUTPUTS]);
        void reset();
        float predictCompute(float lookAheadTime);
        void KalmanStates(float dt);
        void getStates();
        void updateI2C();
        float getRawAngle();

        float angular = 0.0f;
        float gyroRate = 0.0f;
        float accelRaw[3], gyroRaw[3];
        float dt =0;
    private:
        float *_wheelAngular, *_wheelSpeed;
        float _anglePitch = 0.0f;
        float _gyroRate = 0.0f;
        float _accelAngle = 0.0f;
        float _gyrp_rad_s =0.0f;
        float _lastGyroRate = 0.0f;
        float _angularAccel = 0.0f;
        float _K[OUTPUTS];
        unsigned long _lastMicros = 0;

        float P[2][2] = {{1, 0}, {0, 1}}; // Ma trận hiệp phương sai lỗi
        float K_kalman[2];      // Kalman Gain
        uint8_t txBuffer[32];
        uint8_t rxBuffer[32];
};

#endif