#include <utility/DriveDistanceTracker.h>

void DriveDistanceTracker::StartingPosition(double position)
{
    m_startingEncoderTicks = position;
}

double DriveDistanceTracker::GetDistanceInInches(double position)
{
    return (position - m_startingEncoderTicks) * kInchesPerTick;
}
