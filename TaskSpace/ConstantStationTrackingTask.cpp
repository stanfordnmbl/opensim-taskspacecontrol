#include "ConstantStationTrackingTask.h"

using SimTK::Vec3;

using namespace OpenSim;

TaskSpace::ConstantStationTrackingTask::ConstantStationTrackingTask()
{
    constructProperties();
}

void TaskSpace::ConstantStationTrackingTask::constructProperties()
{
    constructProperty_desired_location(Vec3());
}
