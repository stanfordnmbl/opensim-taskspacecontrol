#include "StationTask.h"

#include <Simbody.h>

using SimTK::SpatialVec;
using SimTK::Matrix;
using SimTK::Vec3;
using SimTK::Vector;
using SimTK::Vector_;

using namespace SimTK;

using namespace OpenSim;

Vector TaskSpace::StationTask::generalizedForces(const State& s) const
{
    Vector generalizedForces;
    m_smss->multiplyByStationJacobianTranspose(s,
            m_mobilizedBodyIndex, get_location_in_body(),
            taskSpaceForce(s), generalizedForces);
    return generalizedForces;
}

Matrix TaskSpace::StationTask::jacobian(const State& s) const OVERRIDE_11
{
    Matrix jacobian;
    m_smss->calcStationJacobian(s, m_mobilizedBodyIndex, get_location_in_body(),
            jacobian);
    return jacobian;
}

Vec3 TaskSpace::StationTask::taskSpaceForce(const State& s) const
{
    // TODO turn off compensation.
    // TODO fix matrix multiplication.
    // Had to create this separate variable since Matrix * Vec3 is not defined.
    Vector taskSpaceForce =
        taskSpaceMassMatrix(s) * Vector_<Real>(controlLaw(s)) +
        Vector_<Real>(taskSpaceQuadraticVelocity(s) + taskSpaceGravity(s));
    return Vec3(taskSpaceForce(0), taskSpaceForce(1), taskSpaceForce(2));
}
