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
    volatile float currentSpeed = 0.0f; // steps/s (Current actual velocity)
    volatile float targetSpeed = 0.0f;  // steps/s (Target velocity)
    volatile bool isRunning = false;
    
    // Limit configuration from config.h
    const float min_speed = MIN_SPEED;      // steps/s (Minimum speed to avoid division by zero)
    const float max_speed_limit = MAX_SPEED; // steps/s (Maximum speed limit of the system)
    // Maximum allowed acceleration when changing speed (to prevent step skipping)
    const float max_accel = MAX_ACCEL * rad_to_step; // Convert from rad/s^2 to steps/s^2
    
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

    // --- Main function: Set target velocity ---
    // speed: steps/s
    void setSpeed(float speed) {
        // Limit input speed according to hard configuration
        if (speed > max_speed_limit) speed = max_speed_limit;
        if (speed < -max_speed_limit) speed = -max_speed_limit;
        
        targetSpeed = speed;

        // If motor is stopped and there's a run command, restart timer
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
    
    // Returns actual velocity (currently ramping) not target
    float getSpeed() {
        return currentSpeed;
    }

    // --- Interrupt handler function (Logic conversion v -> f_step) ---
    void handleInterrupt() {
        if (!isRunning) return;

        // 1. Generate pulse
        stepPort->ODR ^= stepBitMask;

        // Calculation logic when completing 1 step (rising edge)
        if ((stepPort->ODR & stepBitMask) != 0) {
            // Update position
            if (currentSpeed > 0) currentPos++;
            else currentPos--;

            // --- IMPORTANT: Ramp algorithm (Create motion profile) ---
            // Calculate dt: time taken for the last step
            float dt = 1.0f / abs(currentSpeed); 
            
            // Calculate maximum allowed velocity change within dt time interval
            // v_next = v_current + a * dt
            float speed_change = max_accel * dt;

            // Adjust currentSpeed towards targetSpeed
            float error = targetSpeed - currentSpeed;

            if (abs(error) <= speed_change) {
                // If error is smaller than acceleration capability, assign directly
                currentSpeed = targetSpeed;
            } else {
                // If error is large, increase/decrease by limited acceleration
                if (error > 0) currentSpeed += speed_change;
                else currentSpeed -= speed_change;
            }

            // --- Handle deadzone and direction reversal ---
            if (abs(currentSpeed) < min_speed) {
                // If target is 0, stop motor
                if (abs(targetSpeed) < min_speed) {
                    currentSpeed = 0;
                    isRunning = false;
                } else {
                    // If reversing direction (e.g., from -100 to +100), keep min_speed to pass through zero
                    currentSpeed = (targetSpeed > 0) ? min_speed : -min_speed;
                }
            }

            // Update rotation direction (Dir pin)
            // Note: Soft directional change logic already handled by currentSpeed sign
            bool logicLevel = (currentSpeed > 0) ? HIGH : LOW;
            if (reverseDir) logicLevel = !logicLevel;
            if (logicLevel) dirPort->BSRR = dirBitMask;
            else dirPort->BSRR = (uint32_t)dirBitMask << 16;
        }

        // 2. Set time for next pulse (Frequency modulation)
        // Delay = 1/2 cycle (because 2 toggles = 1 step)
        // Delay (us) = 1.000.000 / (2 * |speed|)
        uint32_t nextDelay;
        
        if (abs(currentSpeed) < 1.0f) {
             nextDelay = 50000; // Maximum delay limit when speed is near 0
        } else {
             nextDelay = 500000 / abs(currentSpeed);
        }

        // Clamp delay limit to prevent timer errors
        if (nextDelay > 50000) nextDelay = 50000; 
        if (nextDelay < 10) nextDelay = 10;       

        timer->setOverflow(nextDelay, MICROSEC_FORMAT);
    }
};

#endif