#include "StationTask.h"

#include <Simbody.h>

using SimTK::Real;
using SimTK::SpatialVec;
using SimTK::Stage;
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

Vector TaskSpace::StationTask::taskSpaceForces(const State& s) const
{
    // TODO turn off compensation.
    // TODO fix matrix multiplication.
    Vector taskSpaceForce =
        taskSpaceMassMatrix(s) * Vector_<Real>(controlLaw(s)); //TODO +
        // TODO Vector_<Real>(taskSpaceQuadraticVelocity(s) + taskSpaceGravity(s));
     std::cout << "DEBUG StationTask::taskSpaceForce " << taskSpaceForce << std::endl;
    return taskSpaceForce;
}

Matrix TaskSpace::StationTask::jacobian(const State& s) const
{
    Matrix jacobian;
    // TODO cache the MobilizedBodyIndex; make sure it gets updated whenever
    // the body name is updated.
    MobilizedBodyIndex mbi =
        m_model->getBodySet().get(get_body_name()).getIndex();
    m_model->getMatterSubsystem().calcStationJacobian(s, mbi,
            get_location_in_body(), jacobian);
    std::cout << "DEBUG StationTask::jacobian " << jacobian << std::endl;
    return jacobian;
}

Vector TaskSpace::StationTask::taskSpaceQuadraticVelocity(const State& s) const
{
    /** TODO requires Stage::Dynamics...why??
    // b
    // -
    m_model->getMultibodySystem().realize(s, Stage::Dynamics);

    // This loop informed by source for Force::Gravity::getBodyForces.
    int numBodies = m_model->getMatterSubsystem().getNumBodies();
    Vector_<SpatialVec> quadraticVelocitySpatialVecs(numBodies);
    for (MobilizedBodyIndex mbx(1); mbx < numBodies; ++mbx)
    {
        quadraticVelocitySpatialVecs[mbx] =
            m_model->getMatterSubsystem().getTotalCentrifugalForces(s, mbx);
    }
    Vector systemQuadraticVelocity;
    m_model->getMatterSubsystem().multiplyBySystemJacobianTranspose(s,
            quadraticVelocitySpatialVecs, systemQuadraticVelocity);

    // \dot{J} \dot{q} (Khatib's terminology)
    // --------------------------------------
    // TODO cache the MobilizedBodyIndex; make sure it gets updated whenever
    // the body name is updated.
    MobilizedBodyIndex mbi =
        m_model->getBodySet().get(get_body_name()).getIndex();
    Vector jacobianDotTimesU =
        Vector_<Real>(
                m_model->getMatterSubsystem().calcBiasForStationJacobian(s,
                    mbi, get_location_in_body()));

    // \bar{J}^T b - \Lambda \dot{J} \dot{q}
    // -------------------------------------
    return dynamicallyConsistentJacobianInverse(s).transpose() *
        systemQuadraticVelocity - taskSpaceMassMatrix(s) * jacobianDotTimesU;
        */
    return Vector();
}
