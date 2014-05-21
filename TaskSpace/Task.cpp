#include "Task.h"

using SimTK::FactorLU;
using SimTK::Matrix;
using SimTK::MobilizedBodyIndex;
using SimTK::SpatialVec;
using SimTK::State;
using SimTK::Vector;
using SimTK::Vector_;

using namespace OpenSim;

Vector TaskSpace::Task::generalizedForces(const State& s) const
{
    return jacobian(s).transpose() * taskSpaceForces(s);
}

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

    for (unsigned int iST = 0; iST < getNumScalarTasks(); ++iST)
    {
        m_model->getMatterSubsystem().multiplyByMInv(s,
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
    // TODO std::cout << "DEBUG Task::taskSpaceMassMatrix jacobianTranspose " << jacobianTranspose << std::endl;
    Matrix systemMassMatrixInverseTimesJacobianTranspose(
            s.getNU(), getNumScalarTasks());

    for (unsigned int iST = 0; iST < getNumScalarTasks(); ++iST)
    {
        m_model->getMatterSubsystem().multiplyByMInv(s,
                jacobianTranspose.col(iST),
                systemMassMatrixInverseTimesJacobianTranspose.updCol(iST));
    }
    // TODO std::cout << "DEBUG Task::taskSpaceMassMatrix systemMassMatrixInverseTimesJacobianTranspose" << systemMassMatrixInverseTimesJacobianTranspose << std::endl;

    // J A^{-1} J^T
    // -------------
    Matrix taskMassMatrixInverse =
        jac * systemMassMatrixInverseTimesJacobianTranspose;
    // TODO std::cout << "DEBUG Task::taskSpaceMassMatrix taskMassMatrixInverse" << taskMassMatrixInverse << std::endl;

    // (J A^{-1} J^T)^{-1}
    // -------------------
    // TODO compute inverse in a different way; this feels messy.
    FactorLU taskMassMatrixInverseLU(taskMassMatrixInverse);
    Matrix taskMassMatrix(getNumScalarTasks(), getNumScalarTasks());
    taskMassMatrixInverseLU.inverse(taskMassMatrix);

// TODO    std::cout << "DEBUG Task::taskSpaceMassMatrix " << taskMassMatrix << std::endl;
    return taskMassMatrix;
}

Vector TaskSpace::Task::taskSpaceGravity(const State& s) const
{
    Vector systemGravity;
    m_model->getMatterSubsystem().multiplyBySystemJacobianTranspose(s,
            m_model->getGravityForce().getBodyForces(s), systemGravity);
    return dynamicallyConsistentJacobianInverse(s).transpose() * systemGravity;
}
