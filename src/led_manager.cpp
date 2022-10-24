#include "led_manager.h"

void LEDManager::setLED(Led selectedLed, LedStatus ledStatus)
{
    switch (selectedLed)
    {
    case gcs:
        if (_gcsLedStatus != ledStatus)
        {
            _gcsLedStatus = ledStatus;
            switch (_gcsLedStatus)
            {
            case off:
                digitalWrite(gcs, LOW);
                _gcsValue = LOW;
                break;
            case on:
                digitalWrite(gcs, HIGH);
                _gcsValue = HIGH;
                break;
            default:
                break;
            }
        }
        break;
    case wifi:
        if (_wifiLedStatus != ledStatus)
        {
            _wifiLedStatus = ledStatus;
            switch (_wifiLedStatus)
            {
            case off:
                digitalWrite(wifi, LOW);
                _wifiValue = LOW;
                break;
            case on:
                digitalWrite(wifi, HIGH);
                _wifiValue = HIGH;
                break;
            case doubleBlink:
                digitalWrite(wifi, LOW);
                _wifiValue = LOW;
            default:
                break;
            }
        }
        break;
    case air:
        if (_airLedStatus != ledStatus)
        {
            _airLedStatus = ledStatus;
            switch (_airLedStatus)
            {
            case off:
                digitalWrite(air, LOW);
                _airValue = LOW;
                break;
            case on:
                digitalWrite(air, HIGH);
                _airValue = HIGH;
            default:
                break;
            }
        }
        break;
    default:
        break;
    }
}
void LEDManager::blinkLED()
{
    if (millis() >= _timeNextBlink)
    {
        _timeNextBlink = millis() + _cycleTime;
        if (_gcsLedStatus == blink)
        {
            _gcsValue = !_gcsValue;
            digitalWrite(gcs, _gcsValue);
        }
        if (_wifiLedStatus == blink)
        {
            _wifiValue = !_wifiValue;
            digitalWrite(wifi, _wifiValue);
        }
        if (_airLedStatus == blink)
        {
            _airValue = !_airValue;
            digitalWrite(air, _airValue);
        }
    }
}

// double blink on for 200, off for 200, on for 200, off for 600

void LEDManager::doubleBlinkLED()
{
    if (millis() >= _timeNextDoubleBlink && _doubleBlinkCount < 3)
    {
        _doubleBlinkCount++;
        _timeNextDoubleBlink = millis() + (_cycleTime / 3);
        if (_wifiLedStatus == doubleBlink)
        {
            _wifiValue = !_wifiValue;
            digitalWrite(wifi, _wifiValue);
        }
    }
    else if (millis() >= _timeNextDoubleBlink)
    {
        _doubleBlinkCount = 0;
        _timeNextDoubleBlink = millis() + _cycleTime;
        if (_wifiLedStatus == doubleBlink)
        {
            _wifiValue = !_wifiValue;
            digitalWrite(wifi, _wifiValue);
        }
    }
}