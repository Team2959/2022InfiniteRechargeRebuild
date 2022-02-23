#include <utility/StateManager.h>

#include <frc/smartdashboard/SmartDashboard.h>

StateManager::StateManager(Intake& intake, Shooter& shooter, Climb& climb, Vision& vision, Drivetrain& drivetrain, Bling& bling, frc::Joystick& coPilotJoystick)
    : m_intake(intake), m_shooter(shooter), m_climb(climb), m_vision(vision), m_drivetrain(drivetrain), m_bling(bling),
      m_coPilotJoystick(coPilotJoystick)
{
}

void StateManager::OnRobotInit()
{
    UpdateActivePowerCells();
}

void StateManager::OnAutoInit()
{
    Reset();
    StartState(States::Traveling);
    m_powercellsCounted = 3;
    UpdateActivePowerCells();
}

void StateManager::StartState(States state)
{
    m_currentState = state;
    if(m_currentState == States::Firing) FiringInit();
    else if(m_currentState == States::ColorWheel) ColorWheelInit();
    else if(m_currentState == States::Traveling) TravelingInit();
    else if(m_currentState == States::Loading) LoadingInit();
    else if(m_currentState == States::Climbing) ClimbingInit();
}

void StateManager::OnAutoPeriodic()
{
    if(m_currentState == States::Firing) FiringPeriodic();
    else if(m_currentState == States::Traveling) TravelingPeriodic();
    else if(m_currentState == States::Loading) LoadingPeriodic();
}

void StateManager::OnTeleopPeriodic()
{
    ProcessUnjammingButtonPresses();
    if(m_currentState == States::Firing) FiringPeriodic();
    else if(m_currentState == States::ColorWheel) ColorWheelPeriodic();
    else if(m_currentState == States::Traveling) TravelingPeriodic();
    else if(m_currentState == States::Loading) LoadingPeriodic();
    else if(m_currentState == States::Climbing) ClimbingPeriodic();
}

void StateManager::Reset()
{
    ClearPressedAndReleasedOperatorButtons();
}

void StateManager::TravelingInit()
{
    m_intake.SetIntakeSpeed(0);
    m_intake.SetKickerSpeed(0);
    m_intake.SetConveyorSpeed(0);
    
    m_bling.SendMessage(Bling::BlingMessage::red);
    // m_colorWheel.EngageColorWheel(false);
    if (m_powercellsCounted <= 1)
    {
        m_shooter.SetAngle(false);
    }

    // needs shooter to idle speed

    frc::SmartDashboard::PutString("Robot State", "Traveling");
}

void StateManager::TravelingPeriodic() 
{
    // Driving is not included in this because that will be just inside TeleopPeriodic
}

void StateManager::FiringInit() 
{
    m_intake.ProcessStickySwitches();
    m_intake.GetSensorReleased(Intake::SensorLocation::Kicker);
    m_intake.SetIntakeSpeed(0);
    m_intake.SetConveyorSpeed(m_intake.GetConveyorFullSpeed());
    m_intake.SetKickerSpeed(m_intake.GetKickerFullSpeed());

    m_bling.SendMessage(Bling::BlingMessage::popo);

    frc::SmartDashboard::PutString("Robot State", "Firing");
}

void StateManager::FiringPeriodic() 
{
    m_intake.ProcessStickySwitches();
    if (m_intake.GetSensorReleased(Intake::SensorLocation::Kicker))
    {
        m_powercellsCounted--;
        UpdateActivePowerCells();
    }
}

void StateManager::ClimbingInit() 
{
    TravelingInit();

    m_coPilotJoystick.GetRawButtonPressed(kClimbRetract);
    m_coPilotJoystick.GetRawButtonReleased(kClimbRetract);

    m_climb.StartClimb();

    frc::SmartDashboard::PutString("Robot State", "Climbing");
}

void StateManager::ClimbingPeriodic()
{
    m_climb.ProcessClimb(
        m_coPilotJoystick.GetRawButtonPressed(kClimbRetract),
        m_coPilotJoystick.GetRawButtonReleased(kClimbRetract));
}

void StateManager::ColorWheelInit()
{
    TravelingInit();
    // m_coldorWheel.EngageColorWheel(true);

    frc::SmartDashboard::PutString("Robot State", "Color Wheel");
}

void StateManager::ColorWheelPeriodic()
{
    // if (m_colorWheel.IsSpinning())
    // {
    // }
    // else if (m_coPilot.GetRawButtonPressed(kSpinColorWheel))
    // {
    //     m_colorWheel.Spin(true);
    // }
    // else if (m_coPilot.GetRawButtonPressed(kGoToColor))
    // {
    //     m_colorWheel.SpinToColor();
    // }
    // else
    // {
    //     SwitchState(Robot::States::Traveling);
    // }
}

void StateManager::LoadingInit()
{
    m_intake.ProcessStickySwitches();
    m_intake.GetSensorPressed(Intake::SensorLocation::NewPowercell);
    m_intake.GetSensorPressed(Intake::SensorLocation::SecuredPowercell);
    m_shooter.SetAngle(false);
    m_intake.SetConveyorSpeed(0);
    m_intake.SetKickerSpeed(m_intake.GetKickerFullSpeed());
    m_intake.SetIntakeSpeed(m_intake.GetIntakeFullSpeed());
    m_bling.SendMessage(Bling::BlingMessage::hazard);
    m_powercellsCounted = 0;

    frc::SmartDashboard::PutString("Robot State", "Loading");
}

void StateManager::LoadingPeriodic()
{
    m_intake.ProcessStickySwitches();

    if(m_intake.GetSensorPressed(Intake::SensorLocation::NewPowercell))
    {
        if(m_powercellsCounted == 4)
        {
            m_powercellsCounted++;
            StartState(States::Traveling);
            UpdateActivePowerCells();
            return;
        }
        else
        {
            m_intake.SetIntakeSpeed(m_intake.GetIntakeFullSpeed() * 0.5);
            m_intake.SetConveyorSpeed(m_intake.GetConveyorFullSpeedWhenLoading());
        }
    }

    if (!m_intake.GetSensor(Intake::SensorLocation::Kicker))
    {
        m_intake.SetKickerSpeed(0);
    }

    if(m_intake.GetSensorPressed(Intake::SensorLocation::SecuredPowercell))
    {
        m_intake.SetConveyorSpeed(0);
        m_intake.SetIntakeSpeed(m_intake.GetIntakeFullSpeed());
        m_powercellsCounted++;

        if (m_powercellsCounted == 5)
        {
            StartState(States::Traveling);
        }
    }
    /*// If left has ball trigger that side
    if(m_intake.GetLeftBallFlipperSensor())
    {
        // invert whatever state it is in
        m_intake.LeftBallFlipper(!m_intake.GetLeftBallFlipper());
    }
    else if(m_intake.GetRightBallFlipperSensor())
    {
        // invert whatever state it is in
        m_intake.RightBallFlipper(!m_intake.GetRightBallFlipper());
    }

    if(!m_intake.GetLeftBallFlipperSensor())
    {
        m_intake.LeftBallFlipper(true);
    }
    if(!m_intake.GetRightBallFlipperSensor())
    {
        m_intake.RightBallFlipper(true);
    }*/
    
    m_intake.Feed();

    //m_intake.LeftBallFlipper(m_coPilotJoystick.GetRawButton(kClimbExtend));
    //m_intake.RightBallFlipper(m_coPilotJoystick.GetRawButton(kClimbRetract));

    UpdateActivePowerCells();
}

void StateManager::ProcessUnjammingButtonPresses()
{
    if (m_coPilotJoystick.GetRawButtonReleased(kReverseKicker))
    {
        StartState(States::Traveling);
    }
    else if (m_coPilotJoystick.GetRawButtonPressed(kReverseKicker))
    {
        StartState(States::Traveling);
        m_intake.SetIntakeSpeed(-m_intake.GetIntakeFullSpeed());
        m_intake.SetConveyorSpeed(-m_intake.GetConveyorFullSpeedWhenLoading());
        m_intake.SetKickerSpeed(-m_intake.GetKickerFullSpeed());
    }

    if (m_coPilotJoystick.GetRawButtonReleased(kReverseConveyor))
    {
        StartState(States::Traveling);
    }
    else if (m_coPilotJoystick.GetRawButtonPressed(kReverseConveyor))
    {
        StartState(States::Traveling);
        m_intake.SetIntakeSpeed(-m_intake.GetIntakeFullSpeed());
        m_intake.SetConveyorSpeed(-m_intake.GetConveyorFullSpeedWhenLoading());;
    }

    if (m_coPilotJoystick.GetRawButtonReleased(kReverseIntake))
    {
        StartState(States::Traveling);
    }
    else if (m_coPilotJoystick.GetRawButtonPressed(kReverseIntake))
    {
        StartState(States::Traveling);
        m_intake.SetIntakeSpeed(-m_intake.GetIntakeFullSpeed());
    }
}

void StateManager::ClearPressedAndReleasedOperatorButtons()
{
    m_coPilotJoystick.GetTriggerReleased();
    m_coPilotJoystick.GetTriggerPressed();
    m_coPilotJoystick.GetRawButtonPressed(kIntakeToggle);
    m_coPilotJoystick.GetRawButtonPressed(kGoToColor);
    m_coPilotJoystick.GetRawButtonPressed(kEngageColorWheel);
    m_coPilotJoystick.GetRawButtonPressed(kSpinColorWheel);
    m_coPilotJoystick.GetRawButtonPressed(kClimbExtend);
    m_coPilotJoystick.GetRawButtonPressed(kSetAngle);
    m_coPilotJoystick.GetRawButtonPressed(kReverseConveyor);
    m_coPilotJoystick.GetRawButtonReleased(kReverseConveyor);
    m_coPilotJoystick.GetRawButtonPressed(kReverseIntake);
    m_coPilotJoystick.GetRawButtonReleased(kReverseIntake);
    m_coPilotJoystick.GetRawButtonPressed(kReverseKicker);
    m_coPilotJoystick.GetRawButtonReleased(kReverseKicker);
    m_coPilotJoystick.GetRawButtonPressed(kTurnToTarget);
    m_coPilotJoystick.GetRawButtonReleased(kTurnToTarget);
}

void StateManager::UpdateActivePowerCells()
{
    frc::SmartDashboard::PutBoolean("Power Cell 1", m_powercellsCounted >= 1);
    frc::SmartDashboard::PutBoolean("Power Cell 2", m_powercellsCounted >= 2);
    frc::SmartDashboard::PutBoolean("Power Cell 3", m_powercellsCounted >= 3);
    frc::SmartDashboard::PutBoolean("Power Cell 4", m_powercellsCounted >= 4);
    frc::SmartDashboard::PutBoolean("Power Cell 5", m_powercellsCounted >= 5);
}
