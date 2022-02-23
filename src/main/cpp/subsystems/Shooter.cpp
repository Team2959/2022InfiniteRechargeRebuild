#include <subsystems/Shooter.h>
#include <frc/smartdashboard/SmartDashboard.h>

const double kDefaultAutoKp = 0.0004;
const double kDefaultTeleopKp = 0.0006;
const double kDefaultff = 0.000193;
const double kDefaultKd = 0;

Shooter::Shooter()
{    // Have the follower follow the primary except invert 
    // because they are opposite of one another
    m_follower.Follow(m_primary, true);

    ComputeSlopeAndOffset();
}

void Shooter::OnRobotInit()
{
    m_PID.SetP(kDefaultAutoKp);
    m_PID.SetFF(kDefaultff);
    m_PID.SetD(kDefaultKd);
    
    SmartDashboardInit();
}

void Shooter::SmartDashboardInit()
{
    // Debug Enable
    frc::SmartDashboard::PutBoolean(kDebug, m_debugEnable);
    // PID
    frc::SmartDashboard::PutNumber(kPGain, m_PID.GetP());
    frc::SmartDashboard::PutNumber(kIGain, m_PID.GetI());
    frc::SmartDashboard::PutNumber(kFF, m_PID.GetFF());
    frc::SmartDashboard::PutNumber(kIZone, m_PID.GetIZone());
    frc::SmartDashboard::PutNumber(kDGain, m_PID.GetD());
    // Shooter
    frc::SmartDashboard::PutNumber(kTargetSpeed, 0);
    // Close Speed
    frc::SmartDashboard::PutNumber(kCloseSpeed, kCloseSpeedDefault);
    // Applied Output
    frc::SmartDashboard::PutNumber(kAppliedOutput, m_primary.GetAppliedOutput());
    // Min and Max Throttle Speeds
    frc::SmartDashboard::PutNumber(kMaxThrottleSpeed, kMaxThrottleSpeedDefault);
    frc::SmartDashboard::PutNumber(kMinThrottleSpeed, kMinThrottleSpeedDefault);
    // zone speeds
    frc::SmartDashboard::PutNumber(kRedZoneSpeed, kRedZoneSpeedDefault);
    frc::SmartDashboard::PutNumber(kBlueZoneSpeed, kBlueZoneSpeedDefault);
    frc::SmartDashboard::PutNumber(kYellowZoneSpeed, kYellowZoneSpeedDefault);
}

void Shooter::OnRobotPeriodic()
{
    frc::SmartDashboard::PutNumber(kSpeed, GetSpeed());
    frc::SmartDashboard::PutString(kAngle, GetHoodSwitchStateText());
    frc::SmartDashboard::PutNumber(kAppliedOutput, m_primary.GetAppliedOutput());

    m_debugEnable = frc::SmartDashboard::GetBoolean(kDebug, false);
    if (m_debugEnable == false) return;

    m_maxThrottleRange = frc::SmartDashboard::GetNumber(kMaxThrottleSpeed, kMaxThrottleSpeedDefault);
    m_minThrottleRange = frc::SmartDashboard::GetNumber(kMinThrottleSpeed, kMinThrottleSpeedDefault);

    m_redZoneSpeed = frc::SmartDashboard::GetNumber(kRedZoneSpeed, kRedZoneSpeedDefault);
    m_blueZoneSpeed = frc::SmartDashboard::GetNumber(kBlueZoneSpeed, kBlueZoneSpeedDefault);
    m_yellowZoneSpeed = frc::SmartDashboard::GetNumber(kYellowZoneSpeed, kYellowZoneSpeedDefault);

    ComputeSlopeAndOffset();

    // Close Speed
    m_closeSpeed = frc::SmartDashboard::GetNumber(kCloseSpeed, kCloseSpeedDefault);

    // Get the values only once to optimize for speed
    auto currentP = m_PID.GetP();
    auto currentI = m_PID.GetI();
    auto currentFF = m_PID.GetFF();
    auto currentIZone = m_PID.GetIZone();
    auto currentD = m_PID.GetD();

    auto myP = frc::SmartDashboard::GetNumber(kPGain, currentP);
    auto myI = frc::SmartDashboard::GetNumber(kIGain, currentI);
    auto myFF = frc::SmartDashboard::GetNumber(kFF, currentFF);
    auto myIZone = frc::SmartDashboard::GetNumber(kIZone, currentIZone);
    auto myD = frc::SmartDashboard::GetNumber(kDGain, currentD);
    if(fabs(myP - currentP) > kCloseToSameValue)
    {
        m_PID.SetP(myP);
    }
    if(fabs(myI - currentI) > kCloseToSameValue)
    {
        m_PID.SetI(myI);
    }
    if(fabs(myFF - currentFF) > kCloseToSameValue)
    {
        m_PID.SetFF(myFF);
    }
    if(fabs(myIZone - currentIZone) > kCloseToSameValue)
    {
        m_PID.SetIZone(myIZone);
    }
    if(fabs(myD - currentD) > kCloseToSameValue)
    {
        m_PID.SetD(myD);
    }
}

std::string Shooter::GetHoodSwitchStateText()
{
    if(GetAngle())
        return "Far";
    else
        return "Close";    
}

double Shooter::GetSpeed()
{
    return -m_encoder.GetVelocity();
}

bool Shooter::CloseToSpeed()
{
    // return GetSpeed() >= m_targetSpeed - m_closeSpeed;
    return true;
}

void Shooter::ComputeSlopeAndOffset()
{
    m_slopeOfThrottleRange = (m_maxThrottleRange - m_minThrottleRange) / (2 - 0.5);
    m_offsetOfThrottleRange = m_minThrottleRange - (m_slopeOfThrottleRange * 0.5);
}

void Shooter::SetSpeedFromTargetDistance(double distanceInInches)
{
    
}

void Shooter::SetSpeedFromThrottle(double throttlePosition)
{
    throttlePosition += 1;  // shift from -1..1 to 0..2 for range
    // 0..0.5, set shooter speed to 0 
    auto targetSpeed = 0.0;
    // if (throttlePosition >= 0.75)
    // {
    //     // do linear interpolation between minimum and maximum throttle speeds
    //     targetSpeed = (m_slopeOfThrottleRange * throttlePosition) + m_offsetOfThrottleRange; 
    // }
    if (throttlePosition >= 1.75)
    {
        targetSpeed = m_redZoneSpeed;
    }
    else if (throttlePosition >= 1.25)
    {
        targetSpeed = m_blueZoneSpeed;
    }
    else if (throttlePosition >= 0.75)
    {
        targetSpeed = m_yellowZoneSpeed;
    }
    else if (throttlePosition >= 0.25)
    {
        targetSpeed = kWallShotSpeed;
    }
    SetSpeed(targetSpeed);
}

void Shooter::SetSpeed(double speed)
{
    speed = std::fmax(speed, 0);
    frc::SmartDashboard::PutNumber("Throttle Target Speed", speed);
    // invert speed for primary motor direction
    m_targetSpeed = std::fmin(speed, kMaxVelocity);
    m_PID.SetReference(-m_targetSpeed, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
}

void Shooter::SetAngle(bool closeShot)
{
    m_angleAdjuster.Set(closeShot);
}

bool Shooter::GetAngle()
{
    return m_angleAdjuster.Get();
}

void Shooter::SetAutoKp()
{
    m_PID.SetP(kDefaultAutoKp);
}

void Shooter::SetTeleopKp()
{
    m_PID.SetP(kDefaultTeleopKp);
}
