#include <subsystems/Bling.h>

Bling::Bling()
{
    m_last = Bling::BlingMessage::undefined;
}

void Bling::SendMessage(Bling::BlingMessage cmd)
{
    if(cmd == m_last) return;
    m_last = cmd;
    std::string message;
    switch(cmd)
    {
        case Bling::BlingMessage::on:
            message = "ON";
            break;
        case Bling::BlingMessage::off:
            message = "OFF";
            break;
        case Bling::BlingMessage::autonomous:
            message = "AUTO";
            break;
        case Bling::BlingMessage::left:
            message = "LEFT";
            break;
        case Bling::BlingMessage::right:
            message = "RIGHT";
            break;
        case Bling::BlingMessage::reverse:
            message = "REVERSE";
            break;
        case Bling::BlingMessage::brake:
            message = "BRAKE";
            break;
        case Bling::BlingMessage::red:
            message = "RED";
            break;
        case Bling::BlingMessage::blue:
            message = "BLUE";
            break;
        case Bling::BlingMessage::popo:
            message = "POPO";
            break;
        case Bling::BlingMessage::redNums:
            message = "RED_NUMS";
            break;
        case Bling::BlingMessage::blueNums:
            message = "BLUE_NUMS";
            break;
        case Bling::BlingMessage::greenNums:
            message = "GREEN_NUMS";
            break;
        case Bling::BlingMessage::yellowNums:
            message = "YELLOW_NUMS";
            break;
        case Bling::BlingMessage::hazard:
            message = "HAZARD";
            break;
        case Bling::BlingMessage::spin:
            message = "SPIN";
            break;
        default:
            message = "HAZARD";
            break;
    }
    m_serial.Write(message.c_str(), message.length());
    m_serial.Flush();
}
