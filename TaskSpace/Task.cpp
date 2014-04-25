#include "Task.h"

using SimTK::FactorLU;
using SimTK::Matrix;
using SimTK::State;
using SimTK::Vector;

using namespace OpenSim;

Matrix TaskSpace::Task::dynamicallyConsistentJacobianInverse(const State& s)
    const
{
    // J^T \Lambda
    // -----------
    Matrix jacobianTransposeTimesLambda =
        jacobian(s).transpose() * taskSpaceMassMatrix(s);

    // A^{-1} J^T \Lambda
    // ------------------
    Matrix dynConsistentJacobianInverse(s.getNU(), getNumScalarTasks());

    for (unsigned int iST = 0; iST < getNumScalarTasks(); iST++)
    {
        m_smss->multiplyByMInv(s,
                jacobianTransposeTimesLambda.col(iST),
                dynConsistentJacobianInverse.updCol(iST));
    }

    return dynConsistentJacobianInverse;
}

Matrix TaskSpace::Task::taskSpaceMassMatrix(const State& s) const
{
    // A^{-1} J^T
    // -------------
    Matrix jac = jacobian(s);
    Matrix jacobianTranspose = jac.transpose();
    Matrix systemMassMatrixInverseTimesJacobianTranspose(
            s.getNU(), getNumScalarTasks());

    for (unsigned int iST = 0; iST < getNumScalarTasks(); iST++)
    {
        m_smss->multiplyByMInv(s, jacobianTranspose.col(iST),
            systemMassMatrixInverseTimesJacobianTranspose.updCol(iST));
    }

    // J A^{-1} J^T
    // -------------
    Matrix taskMassMatrixInverse =
        jac * systemMassMatrixInverseTimesJacobianTranspose;

    // (J A^{-1} J^T)^{-1}
    // -------------------
    // TODO compute inverse in a different way; this feels messy.
    FactorLU taskMassMatrixInverseLU(taskMassMatrixInverse);
    Matrix taskMassMatrix(getNumScalarTasks(), getNumScalarTasks());
    taskMassMatrixInverseLU.inverse(taskMassMatrix);

    return taskMassMatrix;
}

Vector TaskSpace::Task::taskSpaceQuadraticVelocity(const State& s) const
{
    /* TODO
    // \dot{J} \dot{q} (Khatib's terminology)
    // --------------------------------------
    Vec3 jacobianDotTimesU = m_smss->calcBiasForStationJacobian(s,
            m_mobilizedBodyIndex, get_location_in_body());

    // \bar{J}^T b - \Lambda \dot{J} \dot{q}
    // -------------------------------------
    // TODO cache / copied code.
    Vector systemGravity;
    m_smss->multiplyBySystemJacobianTranspose(s,
            m_model->getGravityForce().getBodyForces(s), systemGravity);

    // See Simbody doxygen documentation of calcResidualForce().
    Vector f_inertial;
    m_smss->calcResidualForce(s, Vector(), Vector_<SpatialVec>(), Vector(),
            Vector(), f_inertial);
    Vector systemQuadraticVelocity = f_inertial - systemGravity;

    return dynamicallyConsistentJacobianInverse(s).transpose() *
        systemQuadraticVelocity - taskSpaceMassMatrix(s) * jacobianDotTimesU;
        */
    return Vector();
}

Vector TaskSpace::Task::taskSpaceGravity(const State& s) const
{
    Vector systemGravity;
    m_smss->multiplyBySystemJacobianTranspose(s,
            m_model->getGravityForce().getBodyForces(s), systemGravity);
    return dynamicallyConsistentJacobianInverse(s).transpose() * systemGravity;
}
