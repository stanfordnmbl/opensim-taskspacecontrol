#include "StationTask.h"

using namespace OpenSim;

Vector TaskSpace::StationTask::generalizedForces(const State& s) const
{
    Vector generalizedForces;
    m_smss.multiplyByStationJacobianTranspose(s,
            m_mobilizedBodyIndex, get_location_in_body(),
            taskSpaceForce(s), returnValue);
    return generalizedForces;
}

Matrix TaskSpace::StationTask::jacobian(const State& s) const OVERRIDE_11 final
{
    Matrix_<Vec3> jacobian;
    m_smss.calcStationJacobian(s, m_mobilizedBodyIndex, get_location_in_body(),
            jacobian);
    return jacobian;
}

Vec3 TaskSpace::StationTask::taskSpaceForce(State& s) const
{
    // TODO turn off compensation.
    return taskSpaceMassMatrix(s) * controlLaw(s) +
        taskSpaceQuadraticVelocity(s) + taskSpaceGravity(s);
}

Vec3 TaskSpace::StationTask::taskSpaceQuadraticVelocity(const State& s) const
{
    // \dot{J} \dot{q} (Khatib's terminology)
    // --------------------------------------
    Vec3 jacobianDotTimesU = m_smss.calcBiasForStationJacobian(s,
            m_mobilizedBodyIndex, get_location_in_body());

    // \bar{J}^T b - \Lambda \dot{J} \dot{q}
    // -------------------------------------
    // TODO cache / copied code.
    Vector systemGravity;
    m_smss.multiplyBySystemJacobianTranspose(s,
            m_model.getGravityForce().getBodyForces(s), systemGravity);

    // See Simbody doxygen documentation of calcResidualForce().
    Vector f_inertial;
    m_smss.calcResidualForce(s, Vector(), Vector_<SpatialVec>(), Vector(),
            Vector(), f_inertial);
    Vector systemQuadraticVelocity = f_inertial - systemGravity;

    return dynamicallyConsistentJacobianInverse(s).transpose() *
        systemQuadraticVelocity - taskSpaceMassMatrix(s) * jacobianDotTimesU;
}

Vec3 TaskSpace::StationTask::taskSpaceGravity(const State& s) const
{
    Vector systemGravity;
    m_smss.multiplyBySystemJacobianTranspose(s,
            m_model.getGravityForce().getBodyForces(s), systemGravity);
    return dynamicallyConsistentJacobianInverse(s).transpose() * systemGravity;
}
