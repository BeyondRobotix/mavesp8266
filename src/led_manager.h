#include "mavesp8266.h"

#ifndef LED_MANAGER_H
#define LED_MANAGER_H

class LEDManager
{
public:
    enum Led
    {
        gcs = 12,
        wifi = 4,
        air = 5
    };
    enum LedStatus
    {
        off,
        on,
        blink,
        doubleBlink
    };
    void setLED(Led selectedLed, LedStatus status);
    void blinkLED();
    void doubleBlinkLED();

private:
    unsigned long _timeNextBlink = 0;       // Time at which the next change in the status light is due
    unsigned long _timeNextDoubleBlink = 0; // Time at which the next double blink is due
    bool _ledsToBlink = false;
    LedStatus _gcsLedStatus = off;
    LedStatus _wifiLedStatus = off;
    LedStatus _airLedStatus = off;
    int _gcsValue = LOW;
    int _wifiValue = LOW;
    int _airValue = LOW;
    int _cycleTime = 600;
    bool _doubleBlinkFlag = false;
};
#endif