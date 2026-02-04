#include "LQR.h"

Sensors::Sensors(const float K[OUTPUTS], float *wheelAngular, float *wheelSpeed)
{
    _wheelAngular = wheelAngular;
    _wheelSpeed = wheelSpeed;
    for (int i = 0; i < OUTPUTS; i++)
    {
        _K[i] = K[i];
    }    
}

void Sensors::intiI2C(){   
    sensors.setSDA(SDA); 
    sensors.setSCL(SCL);
    sensors.begin();
    sensors.setTimeout(5);
    sensors.setClock(i2cFREQ); 
    
    //wake mpu6050 up
    sensors.beginTransmission(MPU6050_ADDR);
    sensors.write(0x6B);
    sensors.write(0);
    sensors.endTransmission(true);
    //DLPF config
    sensors.beginTransmission(MPU6050_ADDR);
    sensors.write(0x1A);
    sensors.write(0x05); 
    sensors.endTransmission(true);
    //Sample rate divider
    sensors.beginTransmission(MPU6050_ADDR);
    sensors.write(0x19);
    sensors.write(0x04); // create 200Hz frequency interrupt
    sensors.endTransmission(true);
    //Interrupt config
    sensors.beginTransmission(MPU6050_ADDR);
    sensors.write(0x38);
    sensors.write(0x01);
    sensors.endTransmission(true);
}

void Sensors::setK(float _K[OUTPUTS]){
    for (int i = 0; i < OUTPUTS; i++)
    {
        _K[i] = K[i];
    }    
}

void Sensors::reset(){
    _anglePitch = 0.0f;
    _gyroRate = 0.0f;
    _lastMicros = micros();
}

void Sensors::updateI2C(){
    // get dt
    unsigned long currentTime = micros();
    dt = ((float)currentTime - (float)_lastMicros) / 1000000.0f;
    _lastMicros = currentTime;
    // Read register
    sensors.beginTransmission(MPU6050_ADDR);
    sensors.write(0x3B);
    sensors.endTransmission(false);

    // Request 14 bytes
    sensors.requestFrom(MPU6050_ADDR, 14);

    // Read data
    if (sensors.available() < 14) return; // Check if data is available (14 bytes)
    int16_t accX = accelRaw[0] = sensors.read() << 8 | sensors.read();
    int16_t accY = accelRaw[1] = sensors.read() << 8 | sensors.read();
    int16_t accZ = accelRaw[2] = sensors.read() << 8 | sensors.read();
    int16_t temp = sensors.read() << 8 | sensors.read();
    int16_t gyroX = gyroRaw[0] = sensors.read() << 8 | sensors.read();
    int16_t gyroY = gyroRaw[1] = sensors.read() << 8 | sensors.read();
    int16_t gyroZ = gyroRaw[2] = sensors.read() << 8 | sensors.read();

    // Calculate angle
    _gyrp_rad_s = (gyroY / 131.0f)*(PI/180.0f);
    _gyroRate = _gyrp_rad_s; // rad/s
    
    _accelAngle = -atan2(accX, -accZ);
}

void Sensors::getStates(){
    updateI2C();
    _anglePitch = MPU_ALPHA * (_anglePitch + trustPrediction*(_gyrp_rad_s * dt)) + (1.0f - MPU_ALPHA) * _accelAngle;
    //float raw_accel = (_gyrp_rad_s - _lastGyroRate) / dt;
    _gyroRate = _gyrp_rad_s;
    angular = _anglePitch + ANGULAR_CALIBRATE;
    gyroRate = _gyroRate;
}

float Sensors::compute(){
    float u = _K[0] * _anglePitch + _K[1] * _gyroRate + _K[2] * (*_wheelAngular) + _K[3] * (*_wheelSpeed);
    u = constrain(u, -MAX_ACCEL, MAX_ACCEL);
    return -u;
}

float Sensors::getRawAngle(){
    return _accelAngle;
}
//END