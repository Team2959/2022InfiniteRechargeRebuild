#pragma once

#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <RobotMap.h>

class Climb
{
private:
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_left {kClimbLeftTalonSrxCanId};
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_right {kClimbRightTalonSrxCanId};

    const double kDefaultKp = 3.0;
    const double kDefaultKi = 0;
    const double kDefaultFf = 0;
    const double kDefaultIzone = 0;
    const double kDefaultCruiseVelocity = 5000;
    const double kDefaultAcceleration = 4500;

    // Smart Dashboard
    const std::string kName = "Climb/";
    const std::string kDebug = kName + "Debug";
    const std::string kPGain = kName + "P Gain";
    const std::string kIGain = kName + "I Gain";
    const std::string kFF = kName + "Feed Forward";
    const std::string kIZone = kName + "I Zone";
    const std::string kCruiseVelocity = kName + "Cruise Velocity";
    const std::string kAcceleration = kName + "Acceleralation";
    const std::string kPosition = kName + "Position";
    const std::string kRightPosition = kName + "Right Position";
    const std::string kVelocity = kName + "Velocity";
    const std::string kTargetPosition = kName + "Target Position";
    const std::string kGoToPosition = kName + "Go To Position";
    const std::string kResetEncoders = kName + "Reset Encoders";
    const std::string kSoftLimitOn = kName + "Soft Limit Enable";

    bool m_debugEnable = false;
    double m_kP = kDefaultKp;
    double m_kI = kDefaultKi;
    double m_kFF = kDefaultFf;
    double m_kIZone = kDefaultIzone;
    double m_cruiseVelocity = kDefaultCruiseVelocity;
    double m_acceleration = kDefaultAcceleration;
    bool m_softLimitEnable = true;

    int m_lastGoToPosition = 0;
    int m_targetPosition = 0;
    int m_delay = 0;

    void StopAndZero();
    void MoveToPosition(int target);
    bool IsAtTargetPosition();
    void StopMotors();

    enum ClimbStates
    {
        Start,
        ReleasePaw,
        ReleaseDelay,
        Extend,
        Retract,
        Retracting,
        RetractAgain,
        MoreRetracting
    };

    ClimbStates m_currentState = Start;

public:
    void OnRobotInit();
    void OnRobotPeriodic();

    void StartClimb();
    void ProcessClimb(bool retractPressed, bool retractReleased);
};
