#include "PriorityLevel.h"

using SimTK::Matrix;
using SimTK::State;
using SimTK::Vector;

using OpenSim::TaskSpace;

Matrix TaskSpace::PriorityLevel::jacobian(const State& s)

Matrix TaskSpace::PriorityLevel::nullspaceProjectionTranspose(const State& s)
{
}

Vector TaskSpace::PriorityLevel::generalizedForce(const State& s)
{
    Vector taskSpaceForce(_numDims);
    for (unsigned int iT = 0; iT < _numTasks; iT++)
    {
        taskSpaceForce[TODO] = _tasks[iT].taskSpaceForce(s);
    }
    return jacobianTranspose(s) * taskSpaceForce;
}
