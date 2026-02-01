#ifndef CONTROL_SPEED_H
#define CONTROL_SPEED_H

#include <Arduino.h>
#include <HardwareTimer.h>
#include "config.h"

class ControlSpeed {
private:
    HardwareTimer *timer;
    uint32_t stepPin;
    uint32_t dirPin;
    GPIO_TypeDef *stepPort;
    uint16_t stepBitMask;
    GPIO_TypeDef *dirPort;
    uint16_t dirBitMask;
    
    volatile long currentPos = 0;
    volatile float currentSpeed = 0.0f; // steps/s (Vận tốc thực tế hiện tại)
    volatile float targetSpeed = 0.0f;  // steps/s (Vận tốc mong muốn)
    volatile bool isRunning = false;
    
    // Cấu hình giới hạn từ config.h
    const float min_speed = MIN_SPEED;      // steps/s (Tốc độ tối thiểu để tránh chia cho 0)
    const float max_speed_limit = MAX_SPEED; // steps/s (Giới hạn tốc độ tối đa của hệ thống)
    // Gia tốc tối đa cho phép khi thay đổi vận tốc (để tránh trượt bước)
    const float max_accel = MAX_ACCEL * rad_to_step; // Chuyển từ rad/s^2 sang steps/s^2
    
    bool reverseDir = 0;

public:
    // Constructor
    ControlSpeed(TIM_TypeDef *timInstance, uint32_t _stepPin, uint32_t _dirPin) {
        timer = new HardwareTimer(timInstance);
        stepPin = _stepPin;
        dirPin = _dirPin;
    }

    void begin() {
        pinMode(stepPin, OUTPUT);
        pinMode(dirPin, OUTPUT);
        
        timer->pause();
        timer->setOverflow(10000, MICROSEC_FORMAT);
        timer->refresh();
        timer->resume();

        stepPort = digitalPinToPort(stepPin);
        stepBitMask = digitalPinToBitMask(stepPin);
        dirPort = digitalPinToPort(dirPin);
        dirBitMask = digitalPinToBitMask(dirPin);
    }

    void setReverseDir(bool dir) {
        reverseDir = dir;
    }

    void attachInterrupt(void (*callback)(void)) {
        timer->attachInterrupt(callback);
    }

    // --- Hàm chính: Đặt vận tốc mục tiêu ---
    // speed: steps/s
    void setSpeed(float speed) {
        // Giới hạn vận tốc đầu vào theo cấu hình cứng
        if (speed > max_speed_limit) speed = max_speed_limit;
        if (speed < -max_speed_limit) speed = -max_speed_limit;
        
        targetSpeed = speed;

        // Nếu động cơ đang dừng mà có lệnh chạy, khởi động lại timer
        if (!isRunning && abs(targetSpeed) > min_speed) {
            currentSpeed = (targetSpeed > 0) ? min_speed : -min_speed;
            isRunning = true;
            timer->setCount(0);
            timer->resume();
        }
    }

    long getPosition() {
        return currentPos;
    }
    
    // Trả về vận tốc thực tế (đang ramp) chứ không phải target
    float getSpeed() {
        return currentSpeed;
    }

    // --- Hàm xử lý ngắt (Logic chuyển đổi v -> f_step) ---
    void handleInterrupt() {
        if (!isRunning) return;

        // 1. Tạo xung
        stepPort->ODR ^= stepBitMask;

        // Logic tính toán khi hoàn thành 1 bước (Cạnh lên)
        if ((stepPort->ODR & stepBitMask) != 0) {
            // Cập nhật vị trí
            if (currentSpeed > 0) currentPos++;
            else currentPos--;

            // --- QUAN TRỌNG: Thuật toán Ramp (Tạo profile chuyển động) ---
            // Tính dt: thời gian thực hiện bước vừa rồi
            float dt = 1.0f / abs(currentSpeed); 
            
            // Tính độ thay đổi vận tốc tối đa cho phép trong khoảng thời gian dt
            // v_next = v_current + a * dt
            float speed_change = max_accel * dt;

            // Điều chỉnh currentSpeed tiến về targetSpeed
            float error = targetSpeed - currentSpeed;

            if (abs(error) <= speed_change) {
                // Nếu sai lệch nhỏ hơn khả năng tăng tốc, gán bằng luôn
                currentSpeed = targetSpeed;
            } else {
                // Nếu sai lệch lớn, tăng/giảm theo gia tốc giới hạn
                if (error > 0) currentSpeed += speed_change;
                else currentSpeed -= speed_change;
            }

            // --- Xử lý vùng chết (Deadzone) và Đảo chiều ---
            if (abs(currentSpeed) < min_speed) {
                // Nếu target về 0, dừng động cơ
                if (abs(targetSpeed) < min_speed) {
                    currentSpeed = 0;
                    isRunning = false;
                } else {
                    // Nếu đang đảo chiều (ví dụ từ -100 lên +100), giữ min_speed để đi qua điểm 0
                    currentSpeed = (targetSpeed > 0) ? min_speed : -min_speed;
                }
            }

            // Cập nhật hướng quay (Dir pin)
            // Lưu ý: Logic đảo chiều mềm (Soft directional change) đã được xử lý bằng dấu của currentSpeed
            bool logicLevel = (currentSpeed > 0) ? HIGH : LOW;
            if (reverseDir) logicLevel = !logicLevel;
            if (logicLevel) dirPort->BSRR = dirBitMask;
            else dirPort->BSRR = (uint32_t)dirBitMask << 16;
        }

        // 2. Cài đặt thời gian cho xung tiếp theo (Frequency modulation)
        // Delay = 1/2 chu kỳ (vì toggle 2 lần = 1 bước)
        // Delay (us) = 1.000.000 / (2 * |speed|)
        uint32_t nextDelay;
        
        if (abs(currentSpeed) < 1.0f) {
             nextDelay = 50000; // Giới hạn delay tối đa nếu tốc độ gần 0
        } else {
             nextDelay = 500000 / abs(currentSpeed);
        }

        // Kẹp giới hạn delay để timer không bị lỗi
        if (nextDelay > 50000) nextDelay = 50000; 
        if (nextDelay < 10) nextDelay = 10;       

        timer->setOverflow(nextDelay, MICROSEC_FORMAT);
    }
};

#endif