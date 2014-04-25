#include "ConstantStationTrackingTask.h"


Vector TaskSpace::ConstantStationTrackingTask::desiredLocation(const State& s)
    const OVERRIDE_11 final
{
    return get_desired_location();
}
