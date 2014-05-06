#include "StationTask.h"

#include <Simbody.h>

using SimTK::Real;
using SimTK::SpatialVec;
using SimTK::Matrix;
using SimTK::Vec3;
using SimTK::Vector;
using SimTK::Vector_;

using namespace OpenSim;

TaskSpace::StationTask::StationTask()
{
    constructProperties();
}

void TaskSpace::StationTask::constructProperties()
{
    // TODO the empty string is potentially confusing.
    constructProperty_body_name("");
    constructProperty_location_in_body(Vec3(0));
}

Vector TaskSpace::StationTask::generalizedForces(const State& s) const
{
    Vector generalizedForces;
    MobilizedBodyIndex idx =
        m_model->getBodySet().get(get_body_name()).getIndex();
    m_model->getMatterSubsystem().multiplyByStationJacobianTranspose(s,
            idx, get_location_in_body(),
            taskSpaceForce(s), generalizedForces);
     std::cout << "DEBUG StationTask::generalizedForces " << generalizedForces << std::endl;
    return generalizedForces;
}

Matrix TaskSpace::StationTask::jacobian(const State& s) const
{
    Matrix jacobian;
    MobilizedBodyIndex idx =
        m_model->getBodySet().get(get_body_name()).getIndex();
    m_model->getMatterSubsystem().calcStationJacobian(s, idx,
            get_location_in_body(), jacobian);
    std::cout << "DEBUG StationTask::jacobian " << jacobian << std::endl;
    return jacobian;
}

Vec3 TaskSpace::StationTask::taskSpaceForce(const State& s) const
{
    // TODO turn off compensation.
    // TODO fix matrix multiplication.
    // Had to create this separate variable since Matrix * Vec3 is not defined.
    Vector taskSpaceForce =
        taskSpaceMassMatrix(s) * Vector_<Real>(controlLaw(s));
     std::cout << "DEBUG StationTask::taskSpaceForce " << taskSpaceForce << std::endl;
    //+
    // TODO    Vector_<Real>(taskSpaceQuadraticVelocity(s) + taskSpaceGravity(s));
    return Vec3(taskSpaceForce(0), taskSpaceForce(1), taskSpaceForce(2));
}
