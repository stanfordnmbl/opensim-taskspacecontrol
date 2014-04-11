#include "PositionTask.h"

Vector PositionTask::taskSpaceForce(State& s) const
{
    return taskSpaceMassMatrix(s) * controlLaw(s) + taskSpaceQuadraticVelocity(s) +
        taskSpaceGravity(s);
}

Matrix PositionTask::nullspaceProjectionTranspose(const State& s) const
{
    // N^T = I - J * {J^#}^T = I - J^T * Lambda * J * A^{-1}
    // TODO can create this at compile-time.
    Matrix identity(_numCoords, _numCoords);
    for (unsigned int i = 0; i < _numCoords; i++)
    {
        identity[i, i] = 1.0;
    }

    // A^{-1}
    Matrix Minv(_numSpeeds, _numSpeeds);
    // Minv is passed by reference; smss puts the inverse mass matrix in Minv.
    smss.calcMInv(s, Minv);

    // J * A^{-1}
    Matrix J_Ainv(_numTaskDim, _numSpeeds);
    for (unsigned int iSpeed = 0; iSpeed < _numSpeeds; iSpeed++)
    {
        multiplyByStationJacobian(s,
                _mobilizedBodyIndex, _positionInBody,
                Minv.col(iSpeed), J_Ainv.updCol(iSpeed));
    }

    // Lambda * J * M^{-1}
    Matrix Lambda_J_Minv = taskSpaceMassMatrix(s) * J_Ainv;

    // J^T * Lambda * J * M^{-1}
    Matrix JT_Lambda_J_Minv(_numSpeeds, _numSpeeds);
    for (unsigned int iSpeed = 0; iSpeed < _numSpeeds; iSpeed++)
    {
        smss.multiplyByStationJacobianTranspose(s,
                _mobilizedBodyIndex, _positionInBody,
                Lambda_J_Minv.col(iSpeed), JT_Lambda_J_Minv.updCol(iSpeed));
    }

    // I - J * {J^#}^T = I - J^T * Lambda * J * M^{-1}
    return identity - JT_Lambda_J_Minv;
}

Vector PositionTask::generalizedForces(const State& s) const
{
    Vector returnValue;
    smss.multiplyByStationJacobianTranspose(s,
            _mobilizedBodyIndex, _positionInBody,
            taskSpaceForce(s), returnValue);
    return returnValue;
}

Matrix taskSpaceMassMatrix(const State& s) const
{
    // Lambda = (J * M^{-1} * J^T)^{-1}

    // J^T
    Matrix J(_numTaskDim, _numSpeeds);
    smss.calcStationJacobian(s, _mobilizedBodyIndex, _positionInBody, J);
    Matrix JT = J.transpose();

    // M^{-1} * J^T
    Matrix Minv_JT(_numSpeeds, _numTaskDim);
    for (unsigned int iTaskDim = 0; iTaskDim < _numTaskDim, iTaskDim++)
    {
        smss.multiplyByMInv(s, JT.col(iTaskDim), Minv_JT.updCol(iTaskDim));
    }

    // J * M^{-1} * J^T
    Matrix J_Minv_JT(_numTaskDim, _numTaskDim);
    for (unsigned int iTaskDim = 0; iTaskDim < _numTaskDim, iTaskDim++)
    {
        // TODO how to assign to a column?
        J_Minv_JT.updCol(iTaskDim) = smss.multiplyByStationJaobian(s,
                _mobilizedBodyIndex, _positionInBody,
                Minv_JT.col(_numTaskDim));
    }
    // TODO best way to get an inverse?
    return J_Minv_JT.inverse();
}

Matrix PositionTask::taskSpaceQuadraticVelocity(const State& s) const
{
}

Matrix PositionTask::taskSpaceGravity(const State& s) const
{
}

Matrix PositionTask::jacobianGeneralizedInverse(const State& s) const
{
    // Jbar = A^{-1} * J^T * Lambda

    // J^T * Lambda
    taskSpace
}

/*
Vec3 PositionTask::multiplyByStationJacobian(const Vector & u)
{
    return smss.multiplyByStationJacobian(s, _mobilizedBodyIndex,
            _positionInBody, u);
}
*/
