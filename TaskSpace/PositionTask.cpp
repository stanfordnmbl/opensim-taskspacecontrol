#include "PositionTask.h"

Vector PositionTask::taskSpaceForce(State& s) const
{
    return taskSpaceMassMatrix * controlLaw(s) + taskSpaceCoriolis(s) +
        taskSpaceGravity(s);
}

Matrix PositionTask::nullspaceProjectionTranspose(const State& s) const
{
    Matrix identity(_numCoords, _numCoords);
    for (unsigned int i = 0; i < _numCoords; i++)
    {
        identity[i, i] = 1.0;
    }
    return identity - jacobian(s).transpose() * jacobianGeneralizedInverse().transpose();

}

Vector generalizedForces(const State& s) const
{
    Vector returnValue;
    smss.multiplyByStationJacobianTranspose(s, _mobilizedBodyIndex,
            _positionInBody, taskSpaceForce(s), returnValue);
    return returnValue;
}
