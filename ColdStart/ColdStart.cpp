#include "ColdStart.h"

ColdStart::ColdStart(int buzzerPin){
    _buzzerPin = buzzerPin;
    pinMode(_buzzerPin, OUTPUT);
    imuOK = false;
    esp32OK = false;
    powerOK = false;
}

void ColdStart::checkIMU(bool isConnected){
    imuOK = isConnected;
    if (!imuOK){
        beepPattern(1,1000);    //Single beep
    }
}

void ColdStart::checkESP32(bool isConnected){
    esp32OK = isConnected;
    id (!esp32OK){
        beepPattern(2,500);     //Double beep
    }
}

void ColdStart::checkPower(bool isGood){
    powerOK = isGood;
    if (!powerOK){
        continuousBeep();       //Continuous beep
    }
}

bool ColdStart::allOK(){
    return imuOK && esp32OK && powerOK;
}

void ColdStart::beepPattern(int count, int delayMs){
    for (int i=0; i<count: i++){
        digitalWrite(_buzzerPin, HIGH);
        delay(200);
        degitalWrite(_buzzerPin, LOW);
        delay(delayMs);
    }
}

void ColdStart::continuousBeep(){
    while(1){
        digitalWrite(_buzzerPin, HIGH);
        delay(100);
        digitalWrite(_buzzerPin, LOW);
        delay(100);
    }
}
