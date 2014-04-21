#include "PriorityLevel.h"

using SimTK::Matrix;
using SimTK::State;
using SimTK::Vector;

using OpenSim::TaskSpace;

PriorityLevel::PriorityLevel()
{
}

Vector TaskSpace::PriorityLevel::generalizedForces(const State& s)
{
    Vector levelGenForces(getNumScalarTasks());

    // Used to write constituent generalized force vectors to the correct
    // place.
    unsigned int STidx = 0;

    for (unsigned int iT = 0; iT < get_tasks().getSize(); iT++)
    {
        unsigned int nST = get_tasks().get(iT).getNumScalarTasks();

        // Write the nST x 1 matrix (vector) to the (STidx, 0) location.
        levelGenForces.updBlock(STidx, 0, nST, 1) =
            get_tasks().get(iT).generalizedForces(s);

        STidx += nST;
    }
    return generalizedForces;
}

Matrix TaskSpace::PriorityLevel::nullspaceProjection(const State& s)
{
    // Build identity matrix.
    Matrix identity(s.getNU(), s.getNU());
    identity.setToZero();
    identity.diag().setTo(1.0);

    return identity - dynamicallyConsistentJacobianInverse(s) * jacobian(s);
}

Matrix TaskSpace::PriorityLevel::jacobian(const State&s s)
{
    Matrix levelJacobian(getNumScalarTasks(), s.getNU());

    // Used to write constituent jacobians to the correct place.
    unsigned int STidx = 0;

    for (unsigned int iT = 0; iT < get_tasks().getSize(); iT++)
    {
        unsigned int nST = get_tasks().get(iT).getNumScalarTasks();

        // Write the nST x NU matrix to the (STidx, 0) location.
        levelJacobian.updBlock(STidx, 0, nST, s.getNU()) =
            get_tasks().get(iT).jacobian(s);

        STidx += nST;
    }
    return levelJacobian;
}

Matrix TaskSpace::PriorityLevel::dynamicallyConsistentJacobianInverse(
        const State& s)
{
    // J^T \Lambda
    // -----------
    Matrix jacobianTransposeTimesLambda =
        jacobian(s).transpose() * taskSpaceMassMatrix(s);

    // A^{-1} J^T \Lambda
    // ------------------
    Matrix dynConsistentJacobianInverse(s.getNU(), getNumScalarTasks());

    for (unsigned int iST = 0; iST < get_tasks().getSize(); iST++)
    {
        m_smss.multiplyByMInv(s,
                jacobianTransposeTimesLambda.col(iST),
                dynConsistentJacobianInverse.updCol(iST));
    }

    return dynConsistentJacobianInverse;
}

Matrix TaskSpace::PriorityLevel::taskSpaceMassMatrix(const State& s)
{
    // A^{-1} J^T
    // -------------
    Matrix jacobian = jacobian(s);
    Matrix jacobianTranspose = jacobian.transpose();
    Matrix systemMassMatrixInverseTimesJacobianTranspose(
            s.getNU(), getNumScalarTasks());

    for (unsigned int iST = 0; iST < getNumScalarTasks(); iST++)
    {
        m_smss.multiplyByMInv(s, jacobianTranspose.col(iST),
            systemMassMatrixInverseTimesJacobianTranspose.updCol(iST));
    }

    // J A^{-1} J^T
    // -------------
    Matrix taskMassMatrixInverse =
        jacobian * systemMassMatrixInverseTimesJacobianTranspose;

    // (J A^{-1} J^T)^{-1}
    // -------------------
    // TODO compute inverse in a different way; this feels messy.
    FactorLU taskMassMatrixInverseLU(taskMassMatrixInverse);
    Matrix taskMassMatrix(s.getNumScalarTasks(), s.getNumScalarTasks());
    taskMassMatrixInverseLU.inverse(taskMassMatrix);

    return taskMassMatrix;
}





