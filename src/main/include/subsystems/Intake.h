#pragma once

#include <frc/DigitalInput.h>
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>
#include <utility/StickySwitch.h>
#include <RobotMap.h>
#include <frc/DoubleSolenoid.h>

class Intake
{
private:
    cwtech::StickySwitch m_newPowercellSensor {kNewPowercellSensor};
    cwtech::StickySwitch m_securedPowercellSensor {kSecuredPowercellSensor};
    cwtech::StickySwitch m_kickerSensor {kKickerSensor};

    cwtech::StickySwitch m_leftFeederSensor{kIntakeLeftFeederSensor};
    cwtech::StickySwitch m_rightFeederSensor{kIntakeRightFeederSensor};

    ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_intake {kIntakeVictorSpxCanId};
    ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_kicker {kKickerVictorSpxCanId};
    ctre::phoenix::motorcontrol::can::WPI_VictorSPX m_conveyor {kConveyorVictorSpxCanId};
    
    frc::DoubleSolenoid m_rightFeeder {frc::PneumaticsModuleType::CTREPCM,2,3};
    frc::DoubleSolenoid m_leftFeeder {frc::PneumaticsModuleType::CTREPCM,4,5};

    // Smart Dashboard
    const std::string kIntakeName = "Intake/";
    const std::string kConveyorName = "Conveyor/";
    const std::string kKickerName = "Kicker/";
    const std::string kDebug = kIntakeName + "Debug";
    const std::string kConveyorSpeed = kConveyorName + "Speed";
    const std::string kKickerSpeed = kKickerName + "Speed";
    const std::string kIntakeSpeed = kIntakeName + "Speed";
    const std::string kIntakeState = kIntakeName + "State";
    const std::string kIntakeFeederState = kIntakeName + "Feeder State";
    const std::string kIntakeFeederRightSensor = kIntakeName + "Right Feeder Sensor";
    const std::string kIntakeFeederLeftSensor = kIntakeName + "Left Feeder Sensor";
    const std::string kConveyorSpeedWhenLoading = kConveyorName + "Speed When Loading";

    bool m_debugEnable = false;

    const double kFullIntakeSpeed = 0.5;//0.75; when we had eccentric intake wheels on side
    const double kFullConveyorSpeed = 0.6;
    const double kFullKickerSpeed = 0.3;
    const double kFullConveyorSpeedWhenLoading = 1.0;
    const int kIntakePushCount = 10;
    const int kIntakeRetractCount = 10;
    double m_intakeSpeed = kFullIntakeSpeed;
    double m_conveyorSpeed = kFullConveyorSpeed;
    double m_conveyorSpeedWhenLoading = kFullConveyorSpeedWhenLoading;
    double m_kickerSpeed = kFullKickerSpeed;
    double m_intakePushCount = kIntakePushCount;
    double m_intakeRetractCount = kIntakeRetractCount;

    std::string GetIntakeStateText();

public:
    enum class SensorLocation
    {
        Kicker,
        NewPowercell,
        SecuredPowercell
    };

    void OnRobotInit();
    void OnRobotPeriodic();
    void ProcessStickySwitches();

    double GetIntakeFullSpeed() const;
    double GetConveyorFullSpeed() const;
    double GetConveyorFullSpeedWhenLoading() const;
    double GetKickerFullSpeed() const;
    bool IsIntakeRunning() const;

    void SetIntakeSpeed(double speed);
    void SetConveyorSpeed(double speed);
    void SetKickerSpeed(double speed);

    enum FeedingState
    {
        Open,
        PushingLeft,
        RetractingLeft,
        PushingRight,
        RetractingRight
    };

    enum FeedingCylinderDirection {
        Opened,
        Closed
    };

    FeedingState m_feedingState = FeedingState::Open;
    long unsigned int m_feedingSteps = 0;
    void Feed();
    void SetFeedingState(FeedingState state);

    bool GetSensor(SensorLocation location);
    bool GetSensorPressed(SensorLocation location);
    bool GetSensorReleased(SensorLocation location);

    void LeftBallFlipper(FeedingCylinderDirection state);
    void RightBallFlipper(FeedingCylinderDirection state);
    bool GetLeftBallFlipper();
    bool GetRightBallFlipper();
    bool GetLeftBallFlipperSensor();
    bool GetRightBallFlipperSensor();
};