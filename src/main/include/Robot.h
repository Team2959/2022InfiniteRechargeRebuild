/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>
#include <fstream>
#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/buttons/JoystickButton.h>
#include <utility/Conditioning.h>
#include <subsystems/Drivetrain.h>
#include <subsystems/Intake.h>
#include <subsystems/Shooter.h>
#include <subsystems/ColorWheel.h>
#include <subsystems/Climb.h>
#include <subsystems/Vision.h>
#include <subsystems/Autonomous.h>
#include <subsystems/Bling.h>
#include <utility/StateManager.h>

class Robot : public frc::TimedRobot
{
private:
  // this variables is used to keep track of the times RobotPeriodic is called
  int m_skips = 0;

  // Joysticks 
  frc::Joystick m_driverJoystick {0};
  frc::Joystick m_coPilot {1};
  frc::Joystick m_leftTankDriveJoystick {2};
  frc::JoystickButton m_quickTurn {&m_driverJoystick, kQuickTurn};

  cwtech::UniformConditioning m_driverSpeedConditioning {};
  cwtech::UniformConditioning m_driverRotationConditioning {};
  
  const double kDefaultDeadband = 0.07;
  const double kDefaultOutputOffset = 0.0;
  const double kDefaultExponent = 3.0;
  const double kDefaultOutputMax = 1.0;
  const double kDefaultRotationOutputMax = 0.8;
  const double kDefaultAutoTurnMultiplier = 0.05;
  const double kDefaultAutoTurnDegrees = 30.0;
  const double kDefaultAutoTurnOffset = 0.01;
  const double kDefaultAutoTurnOutputMax = 1.0;

  bool m_passed2ndStage = false;
  double m_origTx = 0.0;

  // Drivetrain controller
  Drivetrain m_drivetrain {};
  Intake m_intake {};
  Shooter m_shooter {};
  // ColorWheel m_colorWheel {};
  Climb m_climb {};
  Vision m_vision {};
  Bling m_bling {};
  StateManager m_stateManager {m_intake, m_shooter, m_climb, m_vision, m_drivetrain, m_bling, m_coPilot};
  Autonomous m_autonomous {m_stateManager, m_shooter, m_drivetrain};

  const std::string kCameraAngle = "Vision/Camera Angle";
  bool m_usingCurvatureDrive = true;

  std::ofstream m_logFile{};
  frc::SendableChooser<std::string> m_chooser{};
  void StartNewLogFile();
  void Log();
public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
};
