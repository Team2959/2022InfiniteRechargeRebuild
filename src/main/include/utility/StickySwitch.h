#pragma once

#include <frc/DigitalInput.h>

namespace cwtech
{

class StickySwitch : public frc::DigitalInput
{
private:
    bool m_pressed;
    bool m_released;
    bool m_lastRead;
public:
    StickySwitch(int port);

    bool GetPressed();
    bool GetReleased();
    void ProcessForPressed();
};

}
