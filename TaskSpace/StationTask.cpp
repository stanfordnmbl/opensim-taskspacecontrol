#include "StationTask.h"

Vector StationTask::generalizedForces(const State& s) const
{
    Vector generalizedForces;
    m_smss.multiplyByStationJacobianTranspose(s,
            m_mobilizedBodyIndex, get_position_in_body(),
            taskSpaceForce(s), returnValue);
    return generalizedForces;
}

Matrix StationTask::jacobian(const State& s) const OVERRIDE_11 final
{
    Matrix_<Vec3> jacobian;
    m_smss.calcStationJacobian(s, m_mobilizedBodyIndex, get_position_in_body(),
            jacobian);
    return jacobian;
}

Vec3 StationTask::taskSpaceForce(State& s) const
{
    // TODO turn off compensation.
    return taskSpaceMassMatrix(s) * controlLaw(s) +
        taskSpaceQuadraticVelocity(s) + taskSpaceGravity(s);
}

Vec3 StationTask::taskSpaceQuadraticVelocity(const State& s) const
{
    // TODO
}

Vec3 StationTask::taskSpaceGravity(const State& s) const
{
    // TODO
}

Vec3 StationTask::positionOfStationExpressedInGround(const State& s) const
{
    // TODO
}

Vec3 StationTask::velocityOfStationInGroundExpressedInGround(
        const State& s) const
{
    // TODO
}

/*
Vec3 StationTask::multiplyByStationJacobian(const Vector & u)
{
    return smss.multiplyByStationJacobian(s, _mobilizedBodyIndex,
            _positionInBody, u);
}
*/
