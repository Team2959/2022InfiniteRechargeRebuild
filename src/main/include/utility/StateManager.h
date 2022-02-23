#pragma once

#include <subsystems/Intake.h>
#include <subsystems/Shooter.h>
#include <subsystems/ColorWheel.h>
#include <subsystems/Vision.h>
#include <subsystems/Drivetrain.h>
#include <subsystems/Climb.h>
#include <subsystems/Bling.h>
#include <frc/Joystick.h>

enum States
{
    Traveling,
    Firing,
    Climbing,
    ColorWheel,
    Loading,
    Ready,
};

class StateManager
{
private:
    States m_currentState = States::Ready;
    int m_powercellsCounted = 3;
    Intake& m_intake;
    Shooter& m_shooter;
    Climb& m_climb;
    Vision& m_vision;
    Drivetrain& m_drivetrain;
    Bling& m_bling;
    frc::Joystick& m_coPilotJoystick;

    void ProcessUnjammingButtonPresses();
    void ClearPressedAndReleasedOperatorButtons();

    void TravelingInit();
    void TravelingPeriodic();
    void FiringInit();
    void FiringPeriodic();
    void ClimbingInit();
    void ClimbingPeriodic();
    void ColorWheelInit();
    void ColorWheelPeriodic();
    void LoadingInit();
    void LoadingPeriodic();

    void UpdateActivePowerCells();

public:
    StateManager(Intake& intake, Shooter& shooter, Climb& climb, Vision& vision,
        Drivetrain& drivetrain, Bling& bling, frc::Joystick& coPilotJoystick);

    void OnRobotInit();
    void OnAutoInit();
    void OnAutoPeriodic();
    void OnTeleopPeriodic();

    States CurrentState() const { return m_currentState; }
    bool ArePowerCellsEmpty() const { return m_powercellsCounted == 0; }
    void StartState(States state);
    void Reset();
};
