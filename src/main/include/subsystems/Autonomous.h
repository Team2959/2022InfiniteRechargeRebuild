#pragma once

#include <utility/StateManager.h>
#include <subsystems/Shooter.h>
#include <subsystems/Drivetrain.h>
#include <utility/DriveDistanceTracker.h>
#include <frc/smartdashboard/SendableChooser.h>

enum AutoProgram
{
    FireAndForward,
    FireAndBackward,
    RightWithTrench,
    CenterWithTrench,
    LeftWithTrench,
    WallAndFire
};

class Autonomous
{
private:
    StateManager& m_stateManager;
    Shooter& m_shooter;
    Drivetrain& m_driveTrain;

    AutoProgram m_selectedProgram = FireAndForward;
    int m_step = 0;
    double m_autoTurnTargetAngle = 0.0;
    DriveDistanceTracker m_autoDriveDistanceTracker {};
    int m_cycleDelay = 0;
    double m_firingCycleDelay = 125;

    // SmartDashboard auton mode selector
    frc::SendableChooser<std::string> m_chooser;
    frc::SendableChooser<std::string> m_sideChooser;
    const std::string kFireAndForward = "Fire and Forward";
    const std::string kFireAndBackward = "Fire and Backward";
    const std::string kRightWithTrench = "Right with Trench";
    const std::string kCenterWithTrench = "Center with Trench";
    const std::string kLeftWithTrench = "Left with Trench";
    const std::string kWallAndFire = "Wall and Fire";
    const std::string kCenter = "Center";
    const std::string kLeftRight = "Left or Right";
    const std::string kShootDelayCycles = "Shoot Delay Cycles";

    void FireAndForwardPeriodic();
    void FireAndBackwardPeriodic();
    void RightWithTrenchPeriodic();
    void CenterWithTrenchPeriodic();
    void LeftWithTrenchPeriodic();
    void WallAndFirePeriodic();

public:
    Autonomous(StateManager& stateManager, Shooter& shooter, Drivetrain& driveTrain);

    void OnRobotInit();
    void OnAutoInit();
    void Periodic();
};
