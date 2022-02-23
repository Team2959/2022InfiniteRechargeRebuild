#include <subsystems/Intake.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Intake::OnRobotInit()
{
    // Debug Enable
    frc::SmartDashboard::PutBoolean(kDebug, m_debugEnable);
    // Intake
    frc::SmartDashboard::PutNumber(kIntakeSpeed, kFullIntakeSpeed);
    frc::SmartDashboard::PutBoolean(kIntakeFeederRightSensor, false);
    frc::SmartDashboard::PutBoolean(kIntakeFeederLeftSensor, false);
    // Conveyor
    frc::SmartDashboard::PutNumber(kConveyorSpeed, kFullConveyorSpeed);
    frc::SmartDashboard::PutNumber(kConveyorSpeedWhenLoading, kFullConveyorSpeedWhenLoading);
    // Kicker
    frc::SmartDashboard::PutNumber(kKickerSpeed, kFullKickerSpeed);
}

void Intake::OnRobotPeriodic()
{
    m_debugEnable = frc::SmartDashboard::GetBoolean(kDebug, false);
    frc::SmartDashboard::PutBoolean("New Power Cell", GetSensor(Intake::SensorLocation::NewPowercell));
    frc::SmartDashboard::PutBoolean("Secured Power Cell", GetSensor(Intake::SensorLocation::SecuredPowercell));
    frc::SmartDashboard::PutBoolean("Kicker Sensor", GetSensor(Intake::SensorLocation::Kicker));

    frc::SmartDashboard::PutBoolean(kIntakeFeederRightSensor, GetRightBallFlipperSensor());
    frc::SmartDashboard::PutBoolean(kIntakeFeederLeftSensor, GetLeftBallFlipperSensor());
    frc::SmartDashboard::PutNumber(kIntakeFeederState, m_feedingState);
    
    frc::SmartDashboard::PutString(kIntakeState, GetIntakeStateText());
    frc::SmartDashboard::PutNumber("Feeding Extend Time", m_intakePushCount);
    frc::SmartDashboard::PutNumber("Feeding Retract Time", m_intakeRetractCount);

    if (m_debugEnable == false) return;
    m_intakeSpeed = frc::SmartDashboard::GetNumber(kIntakeSpeed, kFullIntakeSpeed);
    m_conveyorSpeed = frc::SmartDashboard::GetNumber(kConveyorSpeed, kFullConveyorSpeed);
    m_conveyorSpeedWhenLoading = frc::SmartDashboard::GetNumber(kConveyorSpeedWhenLoading, kFullConveyorSpeedWhenLoading);
    m_kickerSpeed = frc::SmartDashboard::GetNumber(kKickerSpeed, kFullKickerSpeed);
    m_intakePushCount = frc::SmartDashboard::GetNumber("Feeding Extend Time", m_intakePushCount);
    m_intakeRetractCount = frc::SmartDashboard::GetNumber("Feeding Extend Time", m_intakeRetractCount);
}

std::string Intake::GetIntakeStateText()
{
    if(IsIntakeRunning())
        return "On";
    else
        return "Off";
}

void Intake::ProcessStickySwitches()
{
    m_kickerSensor.ProcessForPressed();
    m_newPowercellSensor.ProcessForPressed();
    m_securedPowercellSensor.ProcessForPressed();
    m_leftFeederSensor.ProcessForPressed();
    m_rightFeederSensor.ProcessForPressed();
}

bool Intake::GetSensor(Intake::SensorLocation location)
{
    switch(location)
    {
    case Intake::SensorLocation::Kicker:
        return m_kickerSensor.Get();
    case Intake::SensorLocation::NewPowercell:
        return m_newPowercellSensor.Get();
    case Intake::SensorLocation::SecuredPowercell:
        return m_securedPowercellSensor.Get();
    }
    return false;
}

bool Intake::GetSensorPressed(Intake::SensorLocation location)
{
    switch(location)
    {
    case Intake::SensorLocation::Kicker:
        return m_kickerSensor.GetPressed();
    case Intake::SensorLocation::NewPowercell:
        return m_newPowercellSensor.GetPressed();
    case Intake::SensorLocation::SecuredPowercell:
        return m_securedPowercellSensor.GetPressed();
    }
    return false;
}

bool Intake::GetSensorReleased(Intake::SensorLocation location)
{
    switch(location)
    {
    case Intake::SensorLocation::Kicker:
        return m_kickerSensor.GetReleased();
    case Intake::SensorLocation::NewPowercell:
        return m_newPowercellSensor.GetReleased();
    case Intake::SensorLocation::SecuredPowercell:
        return m_securedPowercellSensor.GetReleased();
    }
    return false;
}

bool Intake::IsIntakeRunning() const
{
    return m_intake.Get() != 0.0;
}

double Intake::GetIntakeFullSpeed() const
{
    return m_intakeSpeed;
}

void Intake::SetIntakeSpeed(double speed)
{
    m_intake.Set(speed);
}

double Intake::GetConveyorFullSpeed() const
{
    return m_conveyorSpeed;
}

double Intake::GetConveyorFullSpeedWhenLoading() const
{
    return m_conveyorSpeedWhenLoading;
}

void Intake::SetConveyorSpeed(double speed)
{
    m_conveyor.Set(speed);
}

double Intake::GetKickerFullSpeed() const
{
    return m_kickerSpeed;
}

void Intake::SetKickerSpeed(double speed)
{
    m_kicker.Set(-speed);
}

void Intake::LeftBallFlipper(FeedingCylinderDirection state)
{
    auto newState = state == FeedingCylinderDirection::Closed ? frc::DoubleSolenoid::Value::kForward : frc::DoubleSolenoid::Value::kReverse;
    m_leftFeeder.Set(newState);
}

void Intake::RightBallFlipper(FeedingCylinderDirection state)
{
    auto newState = state == FeedingCylinderDirection::Closed ? frc::DoubleSolenoid::Value::kForward : frc::DoubleSolenoid::Value::kReverse;
    m_rightFeeder.Set(newState);
}

bool Intake::GetLeftBallFlipper()
{
    return m_leftFeeder.Get() == frc::DoubleSolenoid::Value::kForward;
}

bool Intake::GetRightBallFlipper()
{
    return m_rightFeeder.Get() == frc::DoubleSolenoid::Value::kForward;
}

bool Intake::GetLeftBallFlipperSensor()
{
    return m_leftFeederSensor.Get();
}

bool Intake::GetRightBallFlipperSensor()
{
    return m_rightFeederSensor.Get();
}

void Intake::SetFeedingState(Intake::FeedingState state) {
    m_feedingSteps = 0;
    m_feedingState = state;
}

void Intake::Feed()
{
    m_feedingSteps++;
    
    switch (m_feedingState) {
    case FeedingState::Open:
        LeftBallFlipper(FeedingCylinderDirection::Opened);
        RightBallFlipper(FeedingCylinderDirection::Opened);

        if (!GetLeftBallFlipperSensor()) {
            SetFeedingState(FeedingState::PushingLeft);
        } else
         if (!GetRightBallFlipperSensor()) {
            SetFeedingState(FeedingState::PushingRight);
        }
        break;
    case FeedingState::PushingLeft:
        LeftBallFlipper(FeedingCylinderDirection::Closed);
        if (m_feedingSteps > m_intakePushCount) {
            SetFeedingState(FeedingState::RetractingLeft);
        }
        break;
    case FeedingState::RetractingLeft:
        LeftBallFlipper(FeedingCylinderDirection::Opened);
        if (m_feedingSteps > m_intakeRetractCount) {
            SetFeedingState(FeedingState::Open);
        }
        break;
    case FeedingState::PushingRight:
        RightBallFlipper(FeedingCylinderDirection::Closed);
        if (m_feedingSteps > m_intakePushCount) {
            SetFeedingState(FeedingState::RetractingRight);
        }
        break;
    case FeedingState::RetractingRight:
        RightBallFlipper(FeedingCylinderDirection::Opened);
        if (m_feedingSteps > m_intakeRetractCount) {
            SetFeedingState(FeedingState::Open);
        }
        break;
    }

    /*if(GetLeftBallFlipperSensor() && m_feedingState != FeedingState::Right)
    {
        m_feedingState = FeedingState::Left;
        m_feedingSteps = 0;
        LeftBallFlipper(false);
    }
    else if(GetRightBallFlipperSensor() && m_feedingState != FeedingState::Left)
    {
        m_feedingState = FeedingState::Right;
        m_feedingSteps = 0;
        RightBallFlipper(false);
    }
    /*if(!GetLeftBallFlipperSensor())
    {
        m_feedingState = FeedingState::Neither;
        LeftBallFlipper(true);
    }
    if(!GetRightBallFlipperSensor())
    {
        m_feedingState = FeedingState::Neither;
        RightBallFlipper(true);
    }
    if(m_feedingState != FeedingState::Neither)
    {
        m_feedingSteps++;
    }
    if(m_feedingSteps == 50)
    {
        m_feedingSteps = 0;
        if(m_feedingState == FeedingState::Left)
        {
            LeftBallFlipper(true);
        }
        else if(m_feedingState == FeedingState::Right)
        {
            RightBallFlipper(true);
        }
        m_feedingState = FeedingState::Neither;
    }*/
}