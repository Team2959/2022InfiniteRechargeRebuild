#pragma once

#include <RobotMap.h>
#include <rev/CANSparkMax.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <frc/Solenoid.h>

#include <thread>

const double kWallShotSpeed = 2600;
const double kInitiationLineSpeed = 1975;
const double kInitiationLineLeftRightSpeed = 2025;

class Shooter
{
private:
    // Hardware
    rev::CANSparkMax m_primary {kShooterPrimary, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANSparkMax m_follower {kShooterFollower, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::SparkMaxRelativeEncoder m_encoder {m_primary.GetEncoder()};
    rev::SparkMaxPIDController m_PID {m_primary.GetPIDController()};

    frc::Solenoid m_angleAdjuster {frc::PneumaticsModuleType::CTREPCM,kShooterAngleAdjusterPcmId};

    // Smart Dashboard
    const std::string kName = "Shooter/";
    const std::string kDebug = kName + "Debug";
    const std::string kPGain = kName + "P Gain";
    const std::string kIGain = kName + "I Gain";
    const std::string kDGain = kName + "D Term";
    const std::string kFF = kName + "Feed Forward";
    const std::string kIZone = kName + "I Zone";
    const std::string kSpeed = kName + "Speed";
    const std::string kTargetSpeed = kName + "Target Speed";
    const std::string kAngle = kName + "Hood Angle";
    const std::string kCloseSpeed = kName + "Close Speed";
    const std::string kAppliedOutput = kName + "Applied Output";
    const std::string kMaxThrottleSpeed = kName + "Max Throttle Speed";
    const std::string kMinThrottleSpeed = kName + "Min Throttle Speed";
    const std::string kRedZoneSpeed = kName + "Red Zone Speed";
    const std::string kBlueZoneSpeed = kName + "Blue Zone Speed";
    const std::string kYellowZoneSpeed = kName + "Yellow Zone Speed";

    const double kMaxVelocity = 4500;
    const double kMaxThrottleSpeedDefault = 4500;
    const double kMinThrottleSpeedDefault = 1500;
    const double kRedZoneSpeedDefault = 3000;
    const double kBlueZoneSpeedDefault = 2750;
    const double kYellowZoneSpeedDefault = 2000;
    const double kCloseSpeedDefault = 100;

    const double kForwardFullSpeed = -1.0;
    const double kSpeedThreshold = 25;
    double m_appliedOutput = 0.0;

    double m_closeSpeed = kCloseSpeedDefault;
    double m_targetSpeed = 0;
    double m_maxThrottleRange = kMaxThrottleSpeedDefault;
    double m_minThrottleRange = kMinThrottleSpeedDefault;
    double m_redZoneSpeed = kRedZoneSpeedDefault;
    double m_blueZoneSpeed = kBlueZoneSpeedDefault;
    double m_yellowZoneSpeed = kYellowZoneSpeedDefault;
    double m_slopeOfThrottleRange = 1;
    double m_offsetOfThrottleRange = 0;

    bool m_debugEnable;

    void SmartDashboardInit();
    void ComputeSlopeAndOffset();
    std::string GetHoodSwitchStateText();

    double GetSpeed();
public:
    Shooter();

    void OnRobotInit();
    void OnRobotPeriodic();

    bool CloseToSpeed();
    void SetSpeedFromThrottle(double throttlePositon);
    void SetSpeedFromTargetDistance(double distanceInInches);
    void SetSpeed(double speed);

    void SetAngle(bool closeShot);
    bool GetAngle();

    void SetAutoKp();
    void SetTeleopKp();
};
