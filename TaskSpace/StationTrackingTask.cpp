#include "StationTrackingTask.h"

using namespace OpenSim;

TaskSpace::StationTrackingTask::StationTrackingTask()
{
    constructProperties();
}

void TaskSpace::StationTrackingTask::constructProperties()
{
    constructProperty_proportional_gain(100);
    constructProperty_derivative_gain(20); 
}

Vec3 TaskSpace::StationTrackingTask::controlLaw(const State& s) const
{
    // \ddot{x}_{des} + k_p (\dot{x}_{des} - \dot{x}) + k_d * (x_{des} - x);
    // TODO rename.
    Vec3 toReturn = desiredAcceleration(s) +
        get_derivative_gain() * (desiredVelocity(s) -
                velocityOfStationInGroundExpressedInGround(s)) +
        get_proportional_gain() * (desiredLocation(s) -
                locationOfStationExpressedInGround(s));
// TODO    std::cout << "DEBUG StationTrackingTask::controlLaw loc in ground " << locationOfStationExpressedInGround(s) << std::endl;
// TODO    std::cout << "DEBUG StationTrackingTask::controlLaw vel in ground " << velocityOfStationInGroundExpressedInGround(s) << std::endl;
// TODO    std::cout << "DEBUG StationTrackingTask::controlLaw " << toReturn << std::endl;
    return toReturn;
}
