#include "subsystems/Drivetrain.h"
#include <Robot.h>

#include "utility/Filesystem.h"

#include <ctime>
#include <unistd.h>
#include <hal/cpp/fpga_clock.h>

#include <frc/smartdashboard/SmartDashboard.h>


Drivetrain::Drivetrain()
{
    m_leftFollower1.Follow(m_leftPrimary);
    m_leftFollower2.Follow(m_leftPrimary);
    m_rightFollower1.Follow(m_rightPrimary);
    m_rightFollower2.Follow(m_rightPrimary);

    m_leftPrimary.SetInverted(false);
    m_rightPrimary.SetInverted(true);

    m_leftEncoder.SetPositionConversionFactor(kConversionFactor);
    m_rightEncoder.SetPositionConversionFactor(kConversionFactor);
    m_leftEncoder.SetVelocityConversionFactor(kConversionFactor / 60.0);
    m_rightEncoder.SetVelocityConversionFactor(kConversionFactor / 60.0);

    
    SetupSparkMax(&m_leftPrimary);
    SetupSparkMax(&m_rightPrimary);
    SetupSparkMax(&m_leftFollower1);
    SetupSparkMax(&m_leftFollower2);
    SetupSparkMax(&m_rightFollower1);
    SetupSparkMax(&m_rightFollower2);
}

void Drivetrain::SetupSparkMax(rev::CANSparkMax* controller)
{
    controller->ClearFaults();
    controller->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    controller->SetSmartCurrentLimit(kCurrentLimit);
    controller->SetOpenLoopRampRate(kOpenLoopRampRate);
}

void Drivetrain::SetVolts(units::volt_t left, units::volt_t right)
{
    m_leftGroup.SetVoltage(left);
    m_rightGroup.SetVoltage(-right);
    m_differentialDrive.Feed();
}

void Drivetrain::CurvatureDrive(double speed, double rotation, bool quickTurn)
{
    m_differentialDrive.CurvatureDrive(speed, rotation, quickTurn);
}

void Drivetrain::TankDrive(double left, double right)
{
    m_differentialDrive.TankDrive(left, right, false);
}


void Drivetrain::OnRobotPeriodic()
{
    auto rot = GetRotation();
    auto left = units::meter_t(m_leftEncoder.GetPosition());
    auto right = units::meter_t(-m_rightEncoder.GetPosition());
    m_lastPosition = m_odometry.Update(rot, left, right);
}

std::string Drivetrain::GetLoggingData()
{
    auto rot = GetRotation();
    auto left = units::meter_t(m_leftEncoder.GetPosition());
    auto right = units::meter_t(-m_rightEncoder.GetPosition());

    auto pos = m_lastPosition;
    
    auto leftV = units::meters_per_second_t(m_leftEncoder.GetVelocity());
    auto rightV = units::meters_per_second_t(-m_rightEncoder.GetVelocity());
    
    std::string data = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(hal::fpga_clock::now().time_since_epoch()).count()) + ","
        + std::to_string(m_navX.GetAngle()) + ","
        + std::to_string(double(left)) + ","
        + std::to_string(double(right)) + ","
        + std::to_string(double(rot.Degrees())) + ","
        + std::to_string(double(pos.X())) + ","
        + std::to_string(double(pos.Y())) + ","
        + std::to_string(double(leftV)) + ","
        + std::to_string(double(rightV)) + ","
        + std::to_string(m_lastLeftSetpoint.to<double>()) + ","
        + std::to_string(m_lastRightSetpoint.to<double>()) + ","
        + "\n";
    return data;
}

double Drivetrain::GetAngle() // units::degree_t
{
    return m_navX.GetAngle();
}

frc::Rotation2d Drivetrain::GetRotation()
{
    return m_navX.GetRotation2d();
}

frc::DifferentialDriveWheelSpeeds Drivetrain::GetWheelSpeeds()
{
    return{
        units::meters_per_second_t(m_leftEncoder.GetVelocity()),
        units::meters_per_second_t(-m_rightEncoder.GetVelocity())
    };
}

frc::Pose2d Drivetrain::GetPose()
{
    return m_odometry.GetPose();
}

void Drivetrain::ResetOdometry(frc::Pose2d pose)
{
    m_leftEncoder.SetPosition(0);
    m_rightEncoder.SetPosition(0);
    m_odometry.ResetPosition(pose, GetRotation());
}

void Drivetrain::CalculateOutput(units::meters_per_second_t left, units::meters_per_second_t right)
{
    m_lastLeftSetpoint = left;
    m_lastRightSetpoint = right;
    auto wheelSpeeds = GetWheelSpeeds();
    auto leftVolts = m_feedForward.Calculate(left);
    auto rightVolts = m_feedForward.Calculate(right);
    auto leftPIDMeasurement = m_leftPIDController.Calculate(wheelSpeeds.left.to<double>(), left.to<double>());
    auto rightPIDMeasurement = m_rightPIDController.Calculate(wheelSpeeds.right.to<double>(), right.to<double>());
    SetVolts(leftVolts + static_cast<units::volt_t>(leftPIDMeasurement),
            rightVolts + static_cast<units::volt_t>(rightPIDMeasurement));
}


void Drivetrain::InitalShowToSmartDashboard()
{
    m_leftPID.SetFF(0.0005);
    m_rightPID.SetFF(0.0005);
    // Debug Enable
    frc::SmartDashboard::PutBoolean(kDebug, m_debugEnable);
    // PID
    frc::SmartDashboard::PutNumber(kPGain, m_leftPID.GetP());
    frc::SmartDashboard::PutNumber(kIGain, m_leftPID.GetI());
    frc::SmartDashboard::PutNumber(kFF, m_leftPID.GetFF());
    frc::SmartDashboard::PutNumber(kIZone, m_leftPID.GetIZone());
    // Turn to Angle parameters
    frc::SmartDashboard::PutNumber(kAutoKp, kDefaultAutoKp);
    frc::SmartDashboard::PutNumber(kAutoLimitAngle, kDefaultLimitAngle);
    frc::SmartDashboard::PutNumber(kAutoMinSpeed, kDefaultMinSpeed);
    frc::SmartDashboard::PutNumber(kNavxAngle, 0);
}

void Drivetrain::UpdateFromSmartDashboard()
{    
    m_debugEnable = frc::SmartDashboard::GetBoolean(kDebug, false);

    if (m_debugEnable == false) return;

    frc::SmartDashboard::PutNumber(kNavxAngle, GetAngle());

    // Get the values only once to optimize for speed
    auto currentP = m_leftPID.GetP();
    auto currentI = m_leftPID.GetI();
    auto currentFF = m_leftPID.GetFF();
    auto currentIZone = m_leftPID.GetIZone();

    auto myP = frc::SmartDashboard::GetNumber(kPGain, currentP);
    auto myI = frc::SmartDashboard::GetNumber(kIGain, currentI);
    auto myFF = frc::SmartDashboard::GetNumber(kFF, currentFF);
    auto myIZone = frc::SmartDashboard::GetNumber(kIZone, currentIZone);
    if(fabs(myP - currentP) > kCloseToSameValue)
    {
        m_rightPID.SetP(myP);
        m_leftPID.SetP(myP);
    }
    if(fabs(myI - currentI) > kCloseToSameValue)
    {
        m_rightPID.SetI(myI);
        m_leftPID.SetI(myI);
    }
    if(fabs(myFF - currentFF) > kCloseToSameValue)
    {
        m_rightPID.SetFF(myFF);
        m_leftPID.SetFF(myFF);
    }
    if(fabs(myIZone - currentIZone) > kCloseToSameValue)
    {
        m_rightPID.SetIZone(myIZone);
        m_leftPID.SetIZone(myIZone);
    }

    m_autoKp = frc::SmartDashboard::GetNumber(kAutoKp, kDefaultAutoKp);
    m_autoLimitAngle = frc::SmartDashboard::GetNumber(kAutoLimitAngle, kDefaultLimitAngle);
    m_autoMinSpeed = frc::SmartDashboard::GetNumber(kAutoMinSpeed, kDefaultMinSpeed);
}

bool Drivetrain::TryTurnToTargetAngle(double tx)
{
    // auto currentAngle = m_navX.GetAngle();
    if (std::fabs(tx) < m_autoLimitAngle)
    {
        CurvatureDrive(0.0, 0.0, false);
        return false;
    }

    // auto rotationMagnitude = tx;//targetAngle - currentAngle;
    auto rotationSpeed = 0.0;
    // is tx negative, which means turn left
    if (tx < 0)
    {
        // turn left -> counter clockwise
        rotationSpeed = std::fmin(-m_autoMinSpeed, tx * m_autoKp);
    }
    else
    {
        // turn right -> clockwise
        rotationSpeed = std::fmax(m_autoMinSpeed, tx * m_autoKp);
    }
    
    CurvatureDrive(0.0, rotationSpeed, true);

    return true;
}

double Drivetrain::GetPosition()
{
    return m_rightEncoder.GetPosition();
}
