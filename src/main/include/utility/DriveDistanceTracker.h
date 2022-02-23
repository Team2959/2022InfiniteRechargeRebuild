#pragma once

#include <subsystems/Drivetrain.h>

class DriveDistanceTracker
{
private:
    double m_startingEncoderTicks = 0;
    const double kInchesPerTick = 0.05155;

public:
    void StartingPosition(double position);
    double GetDistanceInInches(double position);
};
