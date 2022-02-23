#include <subsystems/Autonomous.h>
#include <frc/smartdashboard/SmartDashboard.h>

Autonomous::Autonomous(StateManager& stateManager, Shooter& shooter, Drivetrain& driveTrain) 
    : m_stateManager(stateManager), m_shooter(shooter), m_driveTrain(driveTrain)
{
}

void Autonomous::OnRobotInit()
{
    m_chooser.SetDefaultOption(kFireAndBackward, kFireAndBackward);
    m_chooser.AddOption(kFireAndForward, kFireAndForward);
    // m_chooser.AddOption(kCenterWithTrench, kCenterWithTrench);
    // m_chooser.AddOption(kRightWithTrench, kRightWithTrench);
    // m_chooser.AddOption(kLeftWithTrench, kLeftWithTrench);
    // m_chooser.AddOption(kWallAndFire, kWallAndFire);

    m_sideChooser.SetDefaultOption(kCenter, kCenter);
    m_sideChooser.AddOption(kLeftRight, kLeftRight);

    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
    frc::SmartDashboard::PutData("Auto Modes Sides", &m_sideChooser);
    frc::SmartDashboard::PutNumber(kShootDelayCycles, 125);
}

void Autonomous::OnAutoInit()
{
    m_step = 0;
    m_cycleDelay = 0;
    m_firingCycleDelay = frc::SmartDashboard::GetNumber(kShootDelayCycles, 50);
    m_shooter.SetAngle(false);
    auto side = m_sideChooser.GetSelected();
    if (side == kLeftRight)
    {
        m_shooter.SetSpeed(kInitiationLineLeftRightSpeed);
    }
    else
    {
        m_shooter.SetSpeed(kInitiationLineSpeed);
    }

    // read auto selection from dashboard
    auto selected = m_chooser.GetSelected();
    if (selected == kFireAndForward)
    {
        m_selectedProgram = FireAndForward;
    }
    else if (selected == kFireAndBackward)
    {
        m_selectedProgram = FireAndBackward;
    }
    else if (selected == kCenterWithTrench)
    {
        m_selectedProgram = CenterWithTrench;
    }
    else if (selected == kRightWithTrench)
    {
        m_selectedProgram = RightWithTrench;
        m_shooter.SetSpeed(kInitiationLineLeftRightSpeed);
    }
    else if (selected == kLeftWithTrench)
    {
        m_selectedProgram = LeftWithTrench;
        m_shooter.SetSpeed(kInitiationLineLeftRightSpeed);
    }
    else if (selected == kWallAndFire)
    {
        m_selectedProgram = WallAndFire;
    }
}

void Autonomous::Periodic()
{
    if (m_step == 0)
    {
        if (m_selectedProgram == WallAndFire)
        {
            m_shooter.SetSpeed(kWallShotSpeed);
            m_step++;
        }
        else
        {
            // all programs currently shoot first, may need to change this!!
            // if (m_shooter.CloseToSpeed())
            // waiting 2 1/2 seconds for shooter to get up to speed
            // default to 2 1/2, but configurable in dashboard
            if (m_cycleDelay++ > m_firingCycleDelay)
            {
                m_stateManager.StartState(States::Firing);
                m_step++;
                m_cycleDelay = 0;
            }
            m_driveTrain.CurvatureDrive(0, 0, false);
        }
    }
    else
    {
        switch (m_selectedProgram)
        {
        case FireAndForward:
            FireAndForwardPeriodic();
            break;
        case FireAndBackward:
            FireAndBackwardPeriodic();
            break;
        case RightWithTrench:
            RightWithTrenchPeriodic();
            break;
        case CenterWithTrench:
            CenterWithTrenchPeriodic();
            break;
        case LeftWithTrench:
            LeftWithTrenchPeriodic();
            break;
        case WallAndFire:
            WallAndFirePeriodic();
            break;
        default:
            break;
        }
    }
}

void Autonomous::FireAndForwardPeriodic()
{
    auto speed = 0.0;
    switch (m_step)
    {
    case 1:
        if (m_stateManager.ArePowerCellsEmpty())
        {
            m_step++;
            m_cycleDelay = 0;
        }
        break;
    case 2:
        if (m_cycleDelay++ > 25)
        {
            m_stateManager.StartState(States::Traveling);
            m_autoDriveDistanceTracker.StartingPosition(m_driveTrain.GetPosition());
            speed = 0.1;
            m_step++;
            m_cycleDelay = 0;
        }
        break;
    case 3:
        if (m_cycleDelay++ > 80)
        // if (m_autoDriveDistanceTracker.GetDistanceInInches(m_driveTrain.GetPostion()) >= (1.0 * 12.0))
        {
            m_shooter.SetSpeed(1000);   // idle shooter speed
            m_step++;
        }
        else
        {
            speed = 0.1;
        }
        break;
    }
    m_driveTrain.CurvatureDrive(speed, 0.0, false);
}

void Autonomous::FireAndBackwardPeriodic()
{
    auto speed = 0.0;
    switch (m_step)
    {
    case 1:
        if (m_stateManager.ArePowerCellsEmpty())
        {
            m_step++;
            m_cycleDelay = 0;
        }
        break;
    case 2:
        if (m_cycleDelay++ > 25)
        {
            m_stateManager.StartState(States::Traveling);
            m_autoDriveDistanceTracker.StartingPosition(m_driveTrain.GetPosition());
            speed = -0.15;
            m_step++;
            m_cycleDelay = 0;
        }
        break;
    case 3:
        if (m_cycleDelay++ > 150)
        // if (m_autoDriveDistanceTracker.GetDistanceInInches(m_driveTrain.GetPostion()) <= (-4.0 * 12.0))
        {
            m_shooter.SetSpeed(1000);   // idle shooter speed
            m_step++;
        }
        else
        {
            speed = -0.15;
        }
        break;
    }
    m_driveTrain.CurvatureDrive(speed, 0.0, false);
}

void Autonomous::RightWithTrenchPeriodic()
{
}

void Autonomous::CenterWithTrenchPeriodic()
{
    switch (m_step)
    {
    case 1:
        if (m_stateManager.ArePowerCellsEmpty())
        {
            m_stateManager.StartState(States::Traveling);
            m_shooter.SetSpeed(1000);   // idle shooter speed
            m_autoDriveDistanceTracker.StartingPosition(m_driveTrain.GetPosition());
            m_driveTrain.CurvatureDrive(-0.5, 0, false);
            m_step++;
        }
        break;
    case 2:
        if (m_autoDriveDistanceTracker.GetDistanceInInches(m_driveTrain.GetPosition()) <= (-5.0 * 12.0))
        {
            m_driveTrain.CurvatureDrive(0, 0, false);
            m_autoTurnTargetAngle = m_driveTrain.GetAngle() + 90;
            m_step++;
        }
        else
        {
            m_driveTrain.CurvatureDrive(-0.5, 0, false);
        }
        
        break;
    case 3:
        if (m_driveTrain.TryTurnToTargetAngle(m_autoTurnTargetAngle) == false)
        {
            m_autoTurnTargetAngle = 0;
            // need to add next steps!!
            m_step++;
        }
        break;
    case 4:
        // done, but keep feeding drive
        m_driveTrain.CurvatureDrive(0, 0, false);
        break;
    }
}

void Autonomous::LeftWithTrenchPeriodic()
{
}

void Autonomous::WallAndFirePeriodic()
{
    auto speed = 0.0;
    switch (m_step)
    {
    case 1:
        // drive forward to wall
        m_autoDriveDistanceTracker.StartingPosition(m_driveTrain.GetPosition());
        speed = 0.1;
        m_step++;
        m_cycleDelay++;
        break;
    case 2:
        if (m_cycleDelay++ > 160)
        // if (m_autoDriveDistanceTracker.GetDistanceInInches(m_driveTrain.GetPostion()) >= (6.0 * 12.0))
        {
            speed = 0.1;
        }
        else
        {
            m_stateManager.StartState(States::Firing);
            m_step++;
        }
        break;
    case 3:
        if (m_stateManager.ArePowerCellsEmpty())
        {
            m_step++;
            m_cycleDelay = 0;
        }
        break;
    case 4:
        if (m_cycleDelay++ > 25)
        {
            m_stateManager.StartState(States::Traveling);
            m_shooter.SetSpeed(1000);   // idle shooter speed
            m_step++;
        }
        break;
    case 5:
        // done, but keep feeding drive
        break;
    }

    m_driveTrain.CurvatureDrive(speed, 0, false);
}
