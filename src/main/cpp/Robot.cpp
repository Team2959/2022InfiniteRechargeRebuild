/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "utility/Filesystem.h"

const char* DriverCameraMode = "Driver Camera Mode";

void Robot::RobotInit() 
{
    m_drivetrain.InitalShowToSmartDashboard();
    m_intake.OnRobotInit();
    m_shooter.OnRobotInit();
    // m_colorWheel.OnRobotInit();
    m_climb.OnRobotInit();
    m_vision.OnRopotInit();
    m_autonomous.OnRobotInit();
    m_stateManager.OnRobotInit();

    m_driverSpeedConditioning.SetDeadband(kDefaultDeadband);
    m_driverSpeedConditioning.SetRange(kDefaultOutputOffset, 1.0);
    m_driverSpeedConditioning.SetExponent(kDefaultExponent);

    m_driverRotationConditioning.SetDeadband(kDefaultDeadband);
    m_driverRotationConditioning.SetRange(kDefaultOutputOffset, 1.0);
    m_driverRotationConditioning.SetExponent(kDefaultExponent);

    frc::SmartDashboard::PutNumber("Speed Deadband", kDefaultDeadband);
    frc::SmartDashboard::PutNumber("Speed Output Offset", kDefaultOutputOffset);
    frc::SmartDashboard::PutNumber("Speed Output Max", kDefaultOutputMax);
    frc::SmartDashboard::PutNumber("Speed Exponent", kDefaultExponent);

    frc::SmartDashboard::PutNumber("Rotation Deadband", kDefaultDeadband);
    frc::SmartDashboard::PutNumber("Rotation Output Offset", kDefaultOutputOffset);
    frc::SmartDashboard::PutNumber("Rotation Output Max", kDefaultRotationOutputMax);
    frc::SmartDashboard::PutNumber("Rotation Exponent", kDefaultExponent);

    frc::SmartDashboard::PutBoolean("Curvature Drive", true);

    frc::SmartDashboard::PutBoolean("Update Conditioning", false);

    frc::SmartDashboard::PutBoolean(DriverCameraMode, false);

    frc::SmartDashboard::PutString("Robot State", "Traveling");

    frc::SmartDashboard::PutNumber(kCameraAngle, 23.3);

#define O(n) n, n
    m_chooser.AddOption(O("Undefined"));
    m_chooser.AddOption(O("Slalom"));
    m_chooser.AddOption(O("Light Speed"));
    m_chooser.AddOption(O("Barrel Run"));
    m_chooser.AddOption(O("Bounce"));
#undef O
    frc::SmartDashboard::PutData(&m_chooser);
}

void Robot::RobotPeriodic() 
{
    if (m_skips % 47)
    {
        m_vision.SetCameraAngle(frc::SmartDashboard::GetNumber(kCameraAngle, 23.3));
        m_drivetrain.UpdateFromSmartDashboard();
        if (frc::SmartDashboard::GetBoolean("Update Conditioning", false))
        {
            double ldb = frc::SmartDashboard::GetNumber("Speed Deadband", kDefaultDeadband);
            double loo = frc::SmartDashboard::GetNumber("Speed Output Offset", kDefaultOutputOffset);
            double lex = frc::SmartDashboard::GetNumber("Speed Exponent", kDefaultExponent);
            double lom = frc::SmartDashboard::GetNumber("Speed Output Max", kDefaultOutputMax);

            double rdb = frc::SmartDashboard::GetNumber("Rotation Deadband", kDefaultDeadband);
            double roo = frc::SmartDashboard::GetNumber("Rotation Output Offset", kDefaultOutputOffset);
            double rex = frc::SmartDashboard::GetNumber("Rotation Exponent", kDefaultExponent);
            double rom = frc::SmartDashboard::GetNumber("Rotation Output Max", kDefaultRotationOutputMax);

            m_driverSpeedConditioning.SetDeadband(ldb);
            m_driverSpeedConditioning.SetRange(loo, lom);
            m_driverSpeedConditioning.SetExponent(lex);

            m_driverRotationConditioning.SetDeadband(rdb);
            m_driverRotationConditioning.SetRange(roo, rom);
            m_driverRotationConditioning.SetExponent(rex);
        }
    }
    if (true /*m_skips % 51*/)
    {
        m_shooter.OnRobotPeriodic();
    }
    if (m_skips % 53)
    {
        m_intake.OnRobotPeriodic();
    }
    if (m_skips % 57)
    {
        m_climb.OnRobotPeriodic();
    }

    // m_colorWheel.UpdateColorSensorValues(m_skips);

    if (m_skips % 33)
    {
        m_vision.OnRobotPeriodic();
    }

    // Increment the m_skips variable for counting
    m_skips++;
}


void Robot::DisabledInit() 
{
    m_bling.SendMessage(Bling::BlingMessage::red);
}

void Robot::AutonomousInit()
{
    m_bling.SendMessage(Bling::BlingMessage::autonomous);
    m_shooter.SetAutoKp();
    m_stateManager.OnAutoInit();
    m_autonomous.OnAutoInit();
    m_vision.SetCameraMode(CameraMode::VisionProcessing);
}

void Robot::AutonomousPeriodic()
{
    m_autonomous.Periodic();
    m_stateManager.OnAutoPeriodic();
}

void Robot::TeleopInit()
{
    m_bling.SendMessage(Bling::BlingMessage::spin);
    m_stateManager.Reset();
    m_shooter.SetTeleopKp();
    m_vision.SetCameraMode(CameraMode::VisionProcessing);
    // remove from competition code
    m_intake.SetIntakeSpeed(0);
    m_intake.SetConveyorSpeed(0);
    m_intake.SetKickerSpeed(0);
    m_usingCurvatureDrive = frc::SmartDashboard::GetBoolean("Curvature Drive", true);
}

void Robot::TeleopPeriodic() 
{
    auto    cameraMode{ frc::SmartDashboard::GetBoolean(DriverCameraMode, false) };

    m_vision.SetCameraMode(cameraMode ? CameraMode::Driver : CameraMode::VisionProcessing);

    if (m_coPilot.GetRawButtonPressed(kTurnToTarget))
    {
        // read from camera
        m_origTx = m_vision.GetTargetXAngleDegrees();
    }
    else if (m_coPilot.GetRawButtonReleased(kTurnToTarget))
    {
        m_origTx = 0;
    }
    else if (m_origTx != 0)
    {
        if (m_drivetrain.TryTurnToTargetAngle(m_vision.GetTargetXAngleDegrees()) == false)
        {
            m_origTx = 0;
        }
    }
    else
    {
        if(m_usingCurvatureDrive)
        {
            m_drivetrain.CurvatureDrive(
                m_driverSpeedConditioning.Condition(-m_driverJoystick.GetY()),
                m_driverRotationConditioning.Condition(m_driverJoystick.GetTwist()),
                m_quickTurn.Get());
        }
        else
        {
            m_drivetrain.TankDrive(
                -m_driverSpeedConditioning.Condition(m_leftTankDriveJoystick.GetY()),
                -m_driverSpeedConditioning.Condition(m_driverJoystick.GetY())
            );
        }
        
    }
    
    m_shooter.SetSpeedFromThrottle(m_coPilot.GetThrottle());

    if (m_coPilot.GetRawButtonPressed(kSetAngle))
    {
        m_shooter.SetAngle(!m_shooter.GetAngle());
    }

    // When Firing Done
    if (m_coPilot.GetTriggerReleased())
    {
        m_stateManager.StartState(States::Traveling);
    }
    else if (m_shooter.CloseToSpeed() && m_coPilot.GetTriggerPressed())
    {
        m_stateManager.StartState(States::Firing);
    }

    if (m_coPilot.GetRawButtonPressed(kIntakeToggle))
    {
        if (m_intake.IsIntakeRunning())
        {
            m_stateManager.StartState(States::Traveling);
            m_intake.LeftBallFlipper(Intake::FeedingCylinderDirection::Opened);
            m_intake.RightBallFlipper(Intake::FeedingCylinderDirection::Opened);
        }
        else
        {
            m_stateManager.StartState(States::Loading);
        }
    }

    // if (m_coPilot.GetRawButtonPressed(kEngageColorWheel))
    // {
    //     if (m_colorWheel.IsColorWheelEngaged())
    //     {
    //         m_stateManager.StartState(Robot::States::Traveling);
    //     }
    //     else
    //     {
    //         m_stateManager.StartState(Robot::States::ColorWheel);
    //     }
    // }

    // if (m_coPilot.GetRawButtonPressed(kClimbExtend))
    // {
    //     m_stateManager.StartState(States::Climbing);
    // }

    m_stateManager.OnTeleopPeriodic();
}

void Robot::StartNewLogFile()
{
    if(m_logFile.is_open()) 
        m_logFile.close();
    std::string usbDirectory = GetFirstDirectory("/media");
    std::string filename;
    if(usbDirectory.length() == 0)
    {
        std::string odometryDirectory = "/home/lvuser/odometry";
        filename = GetFirstModifiedFile(odometryDirectory);
        filename = odometryDirectory + "/" + filename;
    }
    else
    {
        usbDirectory = "/media/" + usbDirectory;
        int i;
        std::string odometryPrefix = m_chooser.GetSelected();
        for(i = 0; std::ifstream{usbDirectory + "/" + odometryPrefix + std::to_string(i) + ".csv"}.good(); i++);
        filename = usbDirectory + "/" + odometryPrefix + std::to_string(i) + ".csv";
    }
    // std::cout << "Log File:" << filename << std::endl;
    m_logFile.open(filename);
    if(!m_logFile.good() || !m_logFile.is_open())
    {
        // std::cout << "Log File failed to open" << std::endl;
    }
    m_logFile << "timestamp,angle,left,right,rotation,positionX,positionY,leftVelocity,rightVelocity,leftSetpoint,rightSetpoint\n" << std::flush;
}

void Robot::Log()
{
    std::string data = m_drivetrain.GetLoggingData();
    m_logFile << data << std::flush;
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
