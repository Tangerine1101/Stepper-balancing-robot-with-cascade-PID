#include "LQR.h"

LQR::LQR(const float K[OUTPUTS], float *wheelAngular, float *wheelSpeed)
{
    _wheelAngular = wheelAngular;
    _wheelSpeed = wheelSpeed;
    for (int i = 0; i < OUTPUTS; i++)
    {
        _K[i] = K[i];
    }    
}

void LQR::intiI2C(){   
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

void LQR::setK(float _K[OUTPUTS]){
    for (int i = 0; i < OUTPUTS; i++)
    {
        _K[i] = K[i];
    }    
}

void LQR::reset(){
    _anglePitch = 0.0f;
    _gyroRate = 0.0f;
    _lastMicros = micros();
}

void LQR::updateI2C(){
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

void LQR::getStates(){
    updateI2C();
    _anglePitch = MPU_ALPHA * (_anglePitch + trustPrediction*(_gyrp_rad_s * dt)) + (1.0f - MPU_ALPHA) * _accelAngle;
    //float raw_accel = (_gyrp_rad_s - _lastGyroRate) / dt;
    _gyroRate = _gyrp_rad_s;
    angular = _anglePitch + ANGULAR_CALIBRATE;
    gyroRate = _gyroRate;
}

void LQR::KalmanStates(float dt) {
    
    float raw_accel = (_gyrp_rad_s - _lastGyroRate) / dt;
    _angularAccel = _angularAccel * 0.3f + raw_accel * 0.7f; // Lọc 70% cũ, 30% mới
    _lastGyroRate = _gyrp_rad_s; // Lưu lại cho vòng sau

    _anglePitch += dt * _gyrp_rad_s; 
    
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_gyro * dt;

    // Bước 2: Cập nhật (Update)
    float S = P[0][0] + R_angle;
    K_kalman[0] = P[0][0] / S;
    K_kalman[1] = P[1][0] / S;

    float y = _accelAngle - _anglePitch;
    
    _anglePitch += K_kalman[0] * y;
    _gyroRate   = _gyrp_rad_s; 
    angular = _anglePitch;
    gyroRate = _gyroRate;
    
    // Cập nhật P
    P[0][0] -= K_kalman[0] * P[0][0];
    P[0][1] -= K_kalman[0] * P[0][1];
    P[1][0] -= K_kalman[1] * P[0][0];
    P[1][1] -= K_kalman[1] * P[0][1];
}

float LQR::predictCompute(float lookAheadTime){    
    // Dự đoán góc nghiêng (Bậc 2 - dùng cả gia tốc)
    // Công thức: Góc_tương_lai = Góc + Vận_tốc*T + 0.5*Gia_tốc*T^2
    float pred_Angle = _anglePitch 
                     + (_gyroRate * lookAheadTime) 
                     + (0.5f * _angularAccel * lookAheadTime * lookAheadTime);

    // Dự đoán vận tốc góc (Bậc 1)
    // Công thức: Vận_tốc_tương_lai = Vận_tốc + Gia_tốc*T
    float pred_Rate = _gyroRate + (_angularAccel * lookAheadTime);

    // --- TÍNH LQR VỚI TRẠNG THÁI DỰ BÁO ---
    // Xe sẽ phản ứng như thể nó ĐÃ nghiêng đến vị trí đó rồi -> Phản ứng sớm
    float u = _K[0] * pred_Angle 
            + _K[1] * pred_Rate 
            + _K[2] * (*_wheelAngular) 
            + _K[3] * (*_wheelSpeed);
   
    return -u;
}

float LQR::compute(){
    float u = _K[0] * _anglePitch + _K[1] * _gyroRate + _K[2] * (*_wheelAngular) + _K[3] * (*_wheelSpeed);
    u = constrain(u, -MAX_ACCEL, MAX_ACCEL);
    return -u;
}

float LQR::getRawAngle(){
    return _accelAngle;
}
//END