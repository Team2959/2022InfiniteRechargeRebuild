#pragma once

#include<frc/SerialPort.h>

class Bling
{
public:
    enum BlingMessage
    {
        undefined, // this is a undefined state for m_last to give it an inital value  
        off,
        on,
        autonomous,
        left,
        right,
        reverse,
        brake,
        red,
        blue,
        popo,
        redNums,
        blueNums,
        greenNums,
        yellowNums,
        hazard,
        spin
    };
private:
    frc::SerialPort m_serial{9600, frc::SerialPort::kOnboard};
    BlingMessage m_last;
public:
    Bling();
    void SendMessage(BlingMessage);
};
