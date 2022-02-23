#include <subsystems/Vision.h>
#include <AngleConversion.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/smartdashboard/SmartDashboard.h>

// TO DO:  Specify correct values for CameraAngle, CameraHeight, and TargetHeight
// constexpr double CameraAngleDegrees{ 23.3 };     // Angle in degrees of the vertical elevation of the camera
constexpr double CameraHeight{ 21.5 };         // Height in inches of the camera above the floor.
constexpr double TargetHeight{ 92.0 };         // Height in inches of the target above the floor.

void Vision::OnRopotInit()
{
    auto inst = nt::NetworkTableInstance::GetDefault();
    auto table = inst.GetTable("limelight");
    m_camModeEntry = table->GetEntry("camMode");
    m_tvEntry = table->GetEntry("tv");
    m_txEntry = table->GetEntry("tx");
    m_tyEntry = table->GetEntry("ty");
}

void Vision::OnRobotPeriodic()
{
    // TO DO:  Remove this code once vision testing is complete
    // auto    isTargetValid{ IsTargetValid() };
    // frc::SmartDashboard::PutString("Vision TV Present", isTargetValid ? "Yes" : "No");
    // if(isTargetValid)
    {
        frc::SmartDashboard::PutNumber("Vision TX Angle", GetTargetXAngleDegrees());
        frc::SmartDashboard::PutNumber("Vision TY Angle", GetTargetYAngleDegrees());
        frc::SmartDashboard::PutNumber("Vision Distance", GetTargetDistanceInInches());
    }
    // else
    // {
    //     frc::SmartDashboard::PutString("Vision TX Angle", "");
    //     frc::SmartDashboard::PutString("Vision TY Angle", "");
    //     frc::SmartDashboard::PutString("Vision Distance", "");
    // }
}

// TO DO:  Implement code that calls GetDistanceAngle, GetAngleDistance & GetMotorOutputForAimAndDrive and uses their results
// Adapted from http://docs.limelightvision.io/en/latest/cs_estimating_distance.html
double Vision::GetTargetAngleFromDistance(double distance) const
{
    if(distance == 0.0) // Protect against zero distance and division by zero
        return 0.0;
    return RadiansToDegrees(std::atan((TargetHeight - CameraHeight) / distance)) - m_cameraAngle;  // Do some trigonometry to compute the angle that corresponds to the input distance
}

// Adapted from http://docs.limelightvision.io/en/latest/cs_estimating_distance.html
double Vision::GetTargetDistanceFromAngle(double angle) const
{
    auto    tangent{ std::tan(DegreesToRadians(angle + m_cameraAngle
    
    )) };
    if(tangent == 0.0)  // Avoid any possible division by zero
        return 0.0;
    return (TargetHeight - CameraHeight) / tangent;  // Do some trigonometry to compute the distance that corresponds to the input angle
}

// TO DO:  Specify correct KpAim, KpDistance, & MinAimCommand values
// Adapted from https://docs.limelightvision.io/en/latest/cs_aimandrange.html
std::tuple<double, double> Vision::GetMotorOutputForAimAndDrive(double targetY)
{
    static const double KpAim{ -DegreesToRadians(0.1) };                // These are the coefficients for tuning the response to our target error
    static const double KpDistance{ -DegreesToRadians(0.1) };
    static const double MinAimCommand{ 0.05 };                          // The minimum amount of response if we are turning
    static const double LimitAngle{ DegreesToRadians(1.0) };            // If our angles are within this difference of zero, then we are on target
    auto                targetXAngle{ GetTargetXAngleRadians() };
    auto                heading_error{ 0.0 - targetXAngle };            // Aim for tx == 0.0f
    auto                distance_error{ 0.0};//targetY - GetTargetYAngleRadians() };  // Aim for ty == targetY
    auto                distance_adjust{ KpDistance * distance_error }; // Compute our distance adjustment, and
    double              steering_adjust;                                // Will hold our steering adjustment

    if (targetXAngle > LimitAngle)                                      // If the target is to the right, we need to turn to the left
        steering_adjust = KpAim * heading_error - MinAimCommand;
    else if (targetXAngle < LimitAngle)                                 // If the target is to the left, we need to turn to the right
        steering_adjust = KpAim * heading_error + MinAimCommand;
    else                                                                // Don't turn if +/- 1.0 degrees from crosshairs
        steering_adjust = 0.0f;
    return std::make_tuple(steering_adjust + distance_adjust,  steering_adjust + distance_adjust);  // Apply the distance adjustment to each component
}

double Vision::GetTargetXAngleRadians() const
{
    // if(!IsTargetValid())
    //     return std::nan("");
    return DegreesToRadians(GetTargetXAngleDegrees());
}

double Vision::GetTargetYAngleRadians() const 
{
    // if(!IsTargetValid())
    //     return std::nan("");
    return DegreesToRadians(GetTargetYAngleDegrees());
}

double Vision::GetTargetXAngleDegrees() const
{
    // if(!IsTargetValid())
    //     return std::nan("");
    return m_txEntry.GetDouble(0.0);
}

double Vision::GetTargetYAngleDegrees() const 
{
    // if(!IsTargetValid())
    //     return std::nan("");
    return m_tyEntry.GetDouble(0.0);
}

void Vision::SetCameraAngle(double angle)
{
    m_cameraAngle = angle;
}

CameraMode Vision::GetCameraMode() const
{
    return static_cast<CameraMode>(m_camModeEntry.GetDouble(static_cast<double>(CameraMode::VisionProcessing)));
}

void  Vision::SetCameraMode(CameraMode mode)
{
    m_camModeEntry.SetDouble(static_cast<double>(mode));
}
