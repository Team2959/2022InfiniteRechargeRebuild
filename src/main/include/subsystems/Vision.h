#pragma once

#include <networktables/NetworkTableEntry.h>

enum class CameraMode { VisionProcessing = 0, Driver = 1 };

class Vision
{
private:
    nt::NetworkTableEntry m_camModeEntry;
    nt::NetworkTableEntry m_tvEntry;
    nt::NetworkTableEntry m_txEntry;
    nt::NetworkTableEntry m_tyEntry;

    double GetTargetDistanceFromAngle(double angle) const;
    double GetTargetAngleFromDistance(double distance) const;
    // bool IsTargetValid() const { return m_tvEntry.GetDouble(0.0) != 0.0; }
    // bool IsTargetValid() const { return true; }
    std::tuple<double, double> GetMotorOutputForAimAndDrive(double targetY);

    double m_cameraAngle = 23.3;

    double GetTargetXAngleRadians() const;
    double GetTargetYAngleRadians() const;
    double GetTargetYAngleDegrees() const;

public:
    void OnRopotInit();
    void OnRobotPeriodic();

    void SetCameraAngle(double angle);

    double GetTargetDistanceInInches() const { return GetTargetDistanceFromAngle(GetTargetYAngleDegrees()); }
    double GetTargetXAngleDegrees() const;
    CameraMode GetCameraMode() const;
    void SetCameraMode(CameraMode mode);
};
