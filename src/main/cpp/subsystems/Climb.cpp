#include <subsystems/Climb.h>
#include <frc/smartdashboard/SmartDashboard.h>

const int kReleaseWratchetPawPosition = 600;
const int kExtendPosition = -20000;
const int kRetractPosition = kExtendPosition - 5000;
const int kMaxRetractPosition = kRetractPosition - 4000;
const int kStopDelta = -500;
const int kForwardLimit = 800;

void Climb::OnRobotInit()
{
    m_left.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Relative);
    m_right.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Relative);
    
    m_left.Config_kP(0, m_kP);
    m_left.Config_kI(0, m_kI);
    m_left.Config_kF(0, m_kFF);
    m_left.Config_IntegralZone(0, m_kIZone);
    m_right.Config_kP(0, m_kP);
    m_right.Config_kI(0, m_kI);
    m_right.Config_kF(0, m_kFF);
    m_right.Config_IntegralZone(0, m_kIZone);
    
    m_left.ConfigMotionCruiseVelocity(kDefaultCruiseVelocity, 10);
    m_left.ConfigMotionAcceleration(kDefaultAcceleration, 10);
    m_right.ConfigMotionCruiseVelocity(kDefaultCruiseVelocity, 10);
    m_right.ConfigMotionAcceleration(kDefaultAcceleration, 10);

    m_left.ConfigForwardSoftLimitThreshold(kForwardLimit);
    m_left.ConfigForwardSoftLimitEnable(true);
    m_left.ConfigReverseSoftLimitThreshold(kMaxRetractPosition);
    m_left.ConfigReverseSoftLimitEnable(true);

    m_right.ConfigForwardSoftLimitThreshold(-kMaxRetractPosition);
    m_right.ConfigForwardSoftLimitEnable(true);
    m_right.ConfigReverseSoftLimitThreshold(-kForwardLimit);
    m_right.ConfigReverseSoftLimitEnable(true);

    // Debug Enable
    frc::SmartDashboard::PutBoolean(kDebug, m_debugEnable);
    // PID
    frc::SmartDashboard::PutNumber(kPGain, kDefaultKp);
    frc::SmartDashboard::PutNumber(kIGain, kDefaultKi);
    frc::SmartDashboard::PutNumber(kFF, kDefaultFf);
    frc::SmartDashboard::PutNumber(kIZone, kDefaultIzone);
    // Magic motion
    frc::SmartDashboard::PutNumber(kCruiseVelocity, kDefaultCruiseVelocity);
    frc::SmartDashboard::PutNumber(kAcceleration, kDefaultAcceleration);
    frc::SmartDashboard::PutNumber(kPosition, 0);
    frc::SmartDashboard::PutNumber(kRightPosition, 0);
    frc::SmartDashboard::PutNumber(kVelocity, 0);
    frc::SmartDashboard::PutNumber(kTargetPosition, 0);
    frc::SmartDashboard::PutNumber(kGoToPosition, 0);
    frc::SmartDashboard::PutBoolean(kResetEncoders, false);
    frc::SmartDashboard::PutBoolean(kSoftLimitOn, true);

    StopAndZero();
}

void Climb::OnRobotPeriodic()
{
    auto enableFlag = frc::SmartDashboard::GetBoolean(kDebug, false);
    if (enableFlag == false && enableFlag != m_debugEnable)
    {
        StopMotors();
    }

    m_debugEnable = enableFlag;

    frc::SmartDashboard::PutNumber(kPosition, m_left.GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber(kRightPosition, m_right.GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber(kVelocity, m_left.GetSelectedSensorVelocity());

    if (m_debugEnable == false) return;

    if (frc::SmartDashboard::GetBoolean(kResetEncoders, false))
    {
        StopAndZero();
        frc::SmartDashboard::PutBoolean(kResetEncoders, false);
    }

    auto softLimit = frc::SmartDashboard::GetBoolean(kSoftLimitOn, true);
    if (softLimit != m_softLimitEnable)
    {
        m_softLimitEnable = softLimit;

        m_left.ConfigForwardSoftLimitEnable(m_softLimitEnable);
        m_left.ConfigReverseSoftLimitEnable(m_softLimitEnable);
        m_right.ConfigForwardSoftLimitEnable(m_softLimitEnable);
        m_right.ConfigReverseSoftLimitEnable(m_softLimitEnable);
    }

    auto myP = frc::SmartDashboard::GetNumber(kPGain, m_kP);
    auto myI = frc::SmartDashboard::GetNumber(kIGain, m_kI);
    auto myFF = frc::SmartDashboard::GetNumber(kFF, m_kFF);
    auto myIZone = frc::SmartDashboard::GetNumber(kIZone, m_kIZone);
    auto myCruiseV = frc::SmartDashboard::GetNumber(kCruiseVelocity, kDefaultCruiseVelocity);
    auto myAccel = frc::SmartDashboard::GetNumber(kAcceleration, kDefaultAcceleration);
    if(fabs(myP - m_kP) > kCloseToSameValue)
    {
        m_kP = myP;
        m_left.Config_kP(0, m_kP);
        m_right.Config_kP(0, m_kP);
    }
    if(fabs(myI - m_kI) > kCloseToSameValue)
    {
        m_kI = myI;
        m_left.Config_kI(0, m_kI);
        m_right.Config_kI(0, m_kI);
    }
    if(fabs(myFF - m_kFF) > kCloseToSameValue)
    {
        m_kFF = myFF;
        m_left.Config_kF(0, m_kFF);
        m_right.Config_kF(0, m_kFF);
    }
    if(fabs(myIZone - m_kIZone) > kCloseToSameValue)
    {
        m_kIZone = myIZone;
        m_left.Config_IntegralZone(0, m_kIZone);
        m_right.Config_IntegralZone(0, m_kIZone);
    }
    if(fabs(myCruiseV - m_cruiseVelocity) > kCloseToSameValue)
    {
        m_cruiseVelocity = myCruiseV;
        m_left.ConfigMotionCruiseVelocity(m_cruiseVelocity, 10);
        m_right.ConfigMotionCruiseVelocity(m_cruiseVelocity, 10);
    }
    if(fabs(myAccel - m_acceleration) > kCloseToSameValue)
    {
        m_acceleration = myAccel;
        m_left.ConfigMotionAcceleration(m_acceleration,10);
        m_right.ConfigMotionAcceleration(m_acceleration,10);
    }

    double position = frc::SmartDashboard::GetNumber(kGoToPosition, 0.0);
    if(std::fabs(position - m_lastGoToPosition) > kCloseToSameValue)
    {
        MoveToPosition(position);
        m_lastGoToPosition = position;
    }
}

void Climb::StopAndZero()
{
    m_left.StopMotor();
    m_right.StopMotor();
    m_left.SetSelectedSensorPosition(0,0,0);
    m_right.SetSelectedSensorPosition(0,0,0);
}

void Climb::MoveToPosition(int target)
{
    m_targetPosition = target;
    m_left.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic, m_targetPosition);
    m_right.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic, -m_targetPosition);
    frc::SmartDashboard::PutNumber(kTargetPosition, m_targetPosition);
}

void Climb::StopMotors()
{
    m_left.StopMotor();
    m_right.StopMotor();
}

bool Climb::IsAtTargetPosition()
{
    auto currentPosition = m_left.GetSelectedSensorPosition();
    if (m_targetPosition >= 0)
    {
        return std::abs(currentPosition - m_targetPosition) < 300;
    }

    return std::abs(currentPosition - m_targetPosition) < 1000;
}

void Climb::StartClimb()
{
    m_delay = 0;
    StopAndZero();
    m_currentState = ReleasePaw;
}

void Climb::ProcessClimb(bool retractPressed, bool retractReleased)
{
    switch (m_currentState)
    {
    case ReleasePaw:
        MoveToPosition(kReleaseWratchetPawPosition);
        // if (IsAtTargetPosition())
        {
            m_currentState = ReleaseDelay;
        }
        break;
    case ReleaseDelay:
        if (m_delay++ > 25)
        {
            m_currentState = Extend;
            MoveToPosition(kExtendPosition);
        }
        break;
    case Extend:
        if (IsAtTargetPosition())
        {
            m_currentState = Retract;
            StopMotors();
        }
        break;
    case Retract:
        if (retractPressed)
        {
            m_currentState = Retracting;
            // m_left.Config_kP(0, 3);
            // m_right.Config_kP(0, 3);
            MoveToPosition(kRetractPosition);
        }
        break;
    case Retracting:
        if (retractReleased || IsAtTargetPosition())
        {
            m_currentState = RetractAgain;
            StopMotors();
        }
        break;
    case RetractAgain:
        if (retractPressed)
        {
            m_currentState = MoreRetracting;
        }
        break;
    case MoreRetracting:
        if (retractReleased)
        {
            m_currentState = RetractAgain;
            StopMotors();
        }
        else
        {
            auto targetPosition = std::max((int)m_left.GetSelectedSensorPosition() + 2*kStopDelta, kMaxRetractPosition);
            MoveToPosition(targetPosition);
        }
        break;
    default:
        break;
    }
}
