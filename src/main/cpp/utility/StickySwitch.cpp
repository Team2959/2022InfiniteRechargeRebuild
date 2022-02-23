
#include <utility/StickySwitch.h>

cwtech::StickySwitch::StickySwitch(int port)
    : DigitalInput(port), m_pressed(false)
{
}

bool cwtech::StickySwitch::GetPressed()
{
    if(m_pressed)
    {
        m_pressed = false;
        return true;
    }
    return false;
}

bool cwtech::StickySwitch::GetReleased() 
{
    if(m_released)
    {
        m_released = false;
        return true;
    }
    return false;
}

void cwtech::StickySwitch::ProcessForPressed()
{
    auto currentState = Get();
    if(m_pressed == false && currentState == false && m_lastRead == true)
    {
        m_pressed = true;
    }
    if(m_released == false && currentState == true && m_lastRead == false)
    {
        m_released = true;
    }
    m_lastRead = currentState;
}
