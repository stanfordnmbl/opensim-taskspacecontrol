#include "Task.h"


Vector TaskSpace::Task::taskSpaceForce(State& s) const
{
    return taskSpaceMassMatrix * controlLaw(s) + taskSpaceCoriolis(s) +
        taskSpaceGravity(s);
}
