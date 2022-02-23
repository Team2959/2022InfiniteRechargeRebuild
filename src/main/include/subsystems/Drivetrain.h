#pragma once

#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>

#include <AHRS.h>
#include "RobotMap.h"
#include <rev/CANSparkMax.h>

#include <fstream>

class Drivetrain
{
private:
    const double kOpenLoopRampRate = 0.25;
    const double kCurrentLimit = 50;
    const double kDefaultAutoKp = 0.016;
    const double kDefaultLimitAngle = 0.5;
    const double kDefaultMinSpeed = 0.045;

    rev::CANSparkMax m_leftPrimary{kDrivetrainLeftPrimary, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANSparkMax m_leftFollower1{kDrivetrainLeftFollower1, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANSparkMax m_leftFollower2{kDrivetrainLeftFollower2, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::SparkMaxPIDController m_leftPID{m_leftPrimary.GetPIDController()};
    rev::SparkMaxRelativeEncoder m_leftEncoder{m_leftPrimary.GetEncoder()};

    rev::CANSparkMax m_rightPrimary{kDrivetrainRightPrimary, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANSparkMax m_rightFollower1{kDrivetrainRightFollower1, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANSparkMax m_rightFollower2{kDrivetrainRightFollower2, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::SparkMaxPIDController m_rightPID{m_rightPrimary.GetPIDController()};
    rev::SparkMaxRelativeEncoder m_rightEncoder{m_rightPrimary.GetEncoder()};

    frc::MotorControllerGroup m_leftGroup {m_leftPrimary};
    frc::MotorControllerGroup m_rightGroup {m_rightPrimary};

    frc::DifferentialDrive m_differentialDrive {m_leftGroup, m_rightGroup};

    AHRS m_navX{frc::SPI::kMXP};
    frc::DifferentialDriveOdometry m_odometry{m_navX.GetRotation2d()};

    const units::meter_t kTrackwidth =  0.5461_m; //1.1733265312260803_m; 
    const double kGearboxRatio = 1.0 / 8.67; // One turn of the wheel is 8.67 turns of the motor
    const double kConversionFactor = kGearboxRatio * 5.9 * M_PI * 0.0254; // pi * wheel diameter * inches to meters

    std::ofstream m_logFile{};

    const int kLogInterval = 1;
    int m_steps = 0;
    
    frc2::PIDController m_leftPIDController{Drive::kPDriveVel, 0, 0};
    frc2::PIDController m_rightPIDController{Drive::kPDriveVel, 0, 0};
    frc::SimpleMotorFeedforward<units::meters> m_feedForward{Drive::ks, Drive::kv, Drive::ka};

    void SetupSparkMax(rev::CANSparkMax* controller);


    units::meters_per_second_t m_lastLeftSetpoint;
    units::meters_per_second_t m_lastRightSetpoint;

    frc::Pose2d m_lastPosition;
    
  // Smart Dashboard
  const std::string kName = "Drive/";
  const std::string kDebug = kName + "Debug";
  const std::string kPGain = kName + "P Gain";
  const std::string kIGain = kName + "I Gain";
  const std::string kFF = kName + "Feed Forward";
  const std::string kIZone = kName + "I Zone";
  const std::string kAutoKp = kName + "Auto Turn kP";
  const std::string kAutoLimitAngle = kName + "Auto Turn Limit Angle";
  const std::string kAutoMinSpeed = kName + "Auto Turn Min Speed";
  const std::string kNavxAngle = kName + "Navx Angle";

  bool m_debugEnable;

  double m_autoKp = kDefaultAutoKp;
  double m_autoLimitAngle = kDefaultLimitAngle;
  double m_autoMinSpeed = kDefaultMinSpeed;

public:
    Drivetrain();


    void CurvatureDrive(double speed, double rotation, bool quickTurn);
    void TankDrive(double left, double right);
    
    /**
    * Will be called periodically whenever the CommandScheduler runs.
    */
    void OnRobotPeriodic();

    std::string GetLoggingData();
    void SetVolts(units::volt_t left, units::volt_t right); 
    void ResetOdometry(frc::Pose2d pose);
    frc::Pose2d GetPose();
    frc::DifferentialDriveWheelSpeeds GetWheelSpeeds();
    const frc::DifferentialDriveKinematics kDriveKinematics{kTrackwidth};
    void CalculateOutput(units::meters_per_second_t left, units::meters_per_second_t right);

    double GetPosition();
    double GetAngle();
    frc::Rotation2d GetRotation();
  void InitalShowToSmartDashboard();
  void UpdateFromSmartDashboard();

  bool TryTurnToTargetAngle(double targetAngle);
};
