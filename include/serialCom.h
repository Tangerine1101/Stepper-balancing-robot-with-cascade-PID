#ifndef SERIAL_COMMUNICATION
#define SERIAL_COMMUNICATION

#include <Arduino.h>
#include "config.h"

struct __attribute__((packed)) serialPackage
{
    uint8_t startByte; //0xAA for TX, 0xFE for RX
    float data[Max_arguments]; //{lqr.angular, lqr.gyroRate, wheelPitch, wheelPitchRate, controlVoltage}
    uint8_t checkSum; //XOR checksum

    serialPackage(){
        for (int i =0; i < Max_arguments; i++){
            data[i] = 0;
        }
    }
};

class serialCommunication {
    public:
        serialCommunication(uint8_t startByte);
        void begin();
        void sendBinary(float data[Max_arguments]);
        void sendText(float data[Max_arguments]);
        void clearArgument();
        void getArgument();
        bool requestK();

        float Arguments[OUTPUTS];
        char Indexs[OUTPUTS];
    private:
        void readFrom(unsigned int pos, String Command);

        float privateArg[OUTPUTS];
        char privateIndex[OUTPUTS];
        String incomingCommand = "";
        uint8_t startByte; 
        serialPackage txPackage;
        uint8_t checkSum(serialPackage s);
};

#endif