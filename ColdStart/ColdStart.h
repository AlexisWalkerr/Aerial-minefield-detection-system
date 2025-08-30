#ifndef COLDSTART_H
#define COLDSTART_H
#include <Arduino.h>

class ColdStart{
    public:
        ColdStart(int buzzerPin);
        
        void checkIMU(bool isConnected);
        void checkESP32(bool isConnected);
        void checkPower(bool isGood);

        bool allOk();
        void runStartupCheck();
    private:
        int _buzzerPin;
        bool imuOK, esp32OK, powerOK;

        void beepPattern(int count, int delayMs);
        void continuousBeep();
};

#endif