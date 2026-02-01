#ifndef CONTROL_ACCEL_H
#define CONTROL_ACCEL_H

#include <Arduino.h>
#include <HardwareTimer.h>
#include "config.h"
class ControlAccel {
private:
    HardwareTimer *timer;
    uint32_t stepPin;
    uint32_t dirPin;
    uint32_t channel;
    
    volatile long currentPos = 0;
    volatile float currentSpeed = 0.0f; // steps/s
    volatile float targetAccel = 0.0f;  // steps/s^2
    volatile bool isRunning = false;
    
    const float min_speed = MIN_SPEED;      // steps/s (Tránh chia cho 0)
    const float max_speed = MAX_SPEED;   // steps/s
    bool reverseDir = 0;
    // Chuyển đổi vị trí sang gia tốc (P-controller đơn giản cho moveTo)
    bool positionControlMode = false;
    long targetPosition = 0;
    float Kp_pos = 10.0f; 

public:
    // Constructor
    ControlAccel(TIM_TypeDef *timInstance, uint32_t _stepPin, uint32_t _dirPin) {
        timer = new HardwareTimer(timInstance);
        stepPin = _stepPin;
        dirPin = _dirPin;
        // Lấy channel từ pin map (Cần thiết cho setCaptureCompare nếu dùng PWM, 
        // nhưng ở đây ta dùng ngắt Update tràn nên chỉ cần Timer Instance)
    }

    void begin() {
        pinMode(stepPin, OUTPUT);
        pinMode(dirPin, OUTPUT);
        
        // Cấu hình Timer
        timer->pause();
        // Đặt chế độ ngắt khi tràn (Update Interrupt)
        timer->setOverflow(10000, MICROSEC_FORMAT); // Giá trị khởi tạo
        
        // Gắn hàm callback trong main (vì thư viện class khó gắn static interrupt)
        // Chúng ta sẽ gọi hàm handleInterrupt() từ bên ngoài
        
        timer->refresh();
        timer->resume();
    }
    void setReverseDir(bool dir) {
        reverseDir = dir;
    }
    // Hàm gắn callback (Sử dụng std::function hoặc gọi trực tiếp từ main)
    void attachInterrupt(void (*callback)(void)) {
        timer->attachInterrupt(callback);
    }

    // --- Hàm chính 1: Điều khiển gia tốc (Dùng cho LQR) ---
    // accel: steps/s^2 (Gia tốc mong muốn)
    void controlAccel(float accel) {
        positionControlMode = false;
        targetAccel = accel;
        
        // Nếu động cơ đang dừng và có gia tốc lớn, kích hoạt lại
        if (!isRunning && abs(accel) > 10.0f) {
            currentSpeed = (accel > 0) ? min_speed : -min_speed;
            isRunning = true;
            timer->setCount(0);
            timer->resume();
        }
    }

    // --- Hàm chính 2: Điều khiển vị trí ---
    void moveTo(long position) {
        positionControlMode = true;
        targetPosition = position;
    }
    
    void setCurrentPosition(long pos) {
        currentPos = pos;
    }

    long getPosition() {
        return currentPos;
    }
    
    float getSpeed() {
        return currentSpeed;
    }

    // --- Hàm xử lý trong ngắt (QUAN TRỌNG NHẤT) ---
    // Hàm này phải được gọi trong ngắt Timer
    void handleInterrupt() {
        if (!isRunning) return;

        // 1. Tạo xung
        digitalWrite(stepPin, !digitalRead(stepPin)); // Toggle pin

        // Chỉ xử lý logic khi chân STEP chuyển từ LOW -> HIGH (Hoàn thành 1 bước)
        if (digitalRead(stepPin) == HIGH) {
            // Cập nhật vị trí
            if (currentSpeed > 0) currentPos++;
            else currentPos--;

            // Tính toán vận tốc mới: v = v0 + a*dt
            // dt ở đây là thời gian của bước trước = 1.0 / abs(currentSpeed)
            
            float dt = 1.0f / abs(currentSpeed);
            
            // Nếu đang chế độ vị trí, tính gia tốc dựa trên sai số
            if (positionControlMode) {
                long error = targetPosition - currentPos;
                // P-controller đơn giản chuyển đổi lỗi vị trí -> gia tốc
                targetAccel = error * Kp_pos * currentSpeed; // Gain scaling dynamic
                // Giới hạn gia tốc nếu cần
            }

            // Cập nhật vận tốc
            currentSpeed += targetAccel * dt;

            // Giới hạn vận tốc
            if (currentSpeed > max_speed) currentSpeed = max_speed;
            if (currentSpeed < -max_speed) currentSpeed = -max_speed;

            // Xử lý đảo chiều hoặc dừng
            if (abs(currentSpeed) < min_speed) {
                // Nếu tốc độ quá thấp mà gia tốc ngược chiều -> Đảo chiều
                // Nếu gia tốc cùng chiều -> Giữ min speed để start
                if ((currentSpeed > 0 && targetAccel < 0) || (currentSpeed < 0 && targetAccel > 0)) {
                   // Đảo chiều
                   currentSpeed = -currentSpeed;
                } else if (abs(targetAccel) < 1.0f && positionControlMode && abs(targetPosition - currentPos) < 5) {
                    // Dừng hẳn nếu gần đích
                    isRunning = false;
                    currentSpeed = 0;
                } else {
                     // Giữ tốc độ tối thiểu để không bị chia cho 0
                     currentSpeed = (currentSpeed > 0) ? min_speed : -min_speed;
                }
            }

            // Cập nhật hướng
            if (!reverseDir) digitalWrite(dirPin, (currentSpeed > 0) ? HIGH : LOW);
            else digitalWrite(dirPin, (currentSpeed > 0) ? LOW : HIGH);
        }

        // 2. Cài đặt thời gian cho ngắt tiếp theo
        // Timer đếm micro giây. Toggle cần 2 lần ngắt cho 1 bước.
        // Delay = 1.000.000 / (2 * |speed|) = 500.000 / |speed|
        uint32_t nextDelay = 500000 / abs(currentSpeed);
        
        // Safety check
        if (nextDelay > 50000) nextDelay = 50000; // Max delay (slowest speed)
        if (nextDelay < 10) nextDelay = 10;       // Min delay (fastest speed)

        timer->setOverflow(nextDelay, MICROSEC_FORMAT);
    }
};

#endif