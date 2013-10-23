#include "Controller.h"

using SimTK::Matrix;
using SimTK::State;
using SimTK::Vector;

using OpenSim::Model;

using OpenSim::TaskSpace;

// TODO:
// -Accomodate generalized-coordinate priority levels.

void TaskSpace::Controller::computeControls(const State& s,
        Vector& controls) const
{
    // The control vector.
    // ===================
    // Gamma_i: generalized forces (control) to perfectly achieve tasks in
    //          priority level i. Gamma_i = J_i^T * F_i
    // J_i:     jacobian for all tasks in priority i
    // F_i:     task-space (control) forces to perfectly achieve tasks in
    //          priority levl i. Could be PD control, etc.
    // N_k:     null space projection matrix that ensures a generalized force
    //          does not interfere with task k.
    // N_{0i}:  null space projection matrix that ensures a generalized force
    //          does not interfere with tasks 0 through i.
    //          N_{0i} = prod_{k=0}^i ( N_k )

    // The control:
    // Gamma = Gamma_0 + sum_{i=1}^P ( N_{0(i-1)}^T Gamma_i )
    Vector generalizedForces = _tasks[0].generalizedForces();

    // This matrix will be N_{0(i-1)}^T. Initialize as identity matrix.
    Matrix NT(_numCoords, _numCoords);
    NT.setToZero();
    for (unsigned int i = 0; i < _numCoords; i++) { NT[i, i] = 1.0; }

    for (unsigned int iP = 1; iP < _tasks; iP++)
    {
        /** TODO may make more sense to just ask the task for its jacobian, and
         * to compute the nullspace here. Do we consider generalized-coordiante
         * tasks separately then? That means we'd be responsible for computing
         * its inverse. */
        /** TODO ensure this framework allows us to do posture muscle effort
         * minimization task as described by De Sapio */
        // TODO in which direction do I do this matrix multiplication?
        NT = NT * _tasks[iP - 1].nullspaceProjectionTranspose(s);
        Vector Gamma_iP = _tasks[iP].generalizedForces(s);

        generalizedForces += NT * Gamma_iP;
    }
    
    // TODO more efficient version of the loop above:
    // Gamma = Gamma_0 + N_0^T * (Gamma_1 + N_1^T * (Gamma_2 + N_2^T * (...)))
    // We compute the inner parentheses first. This is like how it's more
    // efficient to compute the polynomial a + bx + cx^2 + dx^3 as:
    //      a + x(b + x(c + xd))
    /*
    Vector generalizedForces(_numCoords);
    for (unsigned int iP = _numPriorityLevels - 1; iP > 0; iP--)
    {
        // TODO in which direction do I do this matrix multiplication?
        NT = _tasks[iP - 1].nullspaceProjectionTranspose(s);
        Vector Gamma_iP = _tasks[iP].generalizedForce(s);

        generalizedForces = NT * (Gamma_iP + generalizedForces);
    }
    // The highest-priority level doesn't get filtered by a nullspace
    // projection.
    generalizedForces = _tasks[0].generalizedForce() + generalizedForces;
    */

    // Send control signals to CoordinateActuators.
    // ============================================
    // TODO what if a coordinate is disabled? etc. Many issues to consider here.
    // What if an actuator is disabled?
    // What if there is no actuator with name coordName?
    // What if the actuator with coordName is not a CoordinateActuator?
    // What if there are quaternions (num coords != num speeds)?
    for (unsigned int iCoord = 0; iCoord < _numCoords; iCoord++)
    {
        Vector thisActuatorsControls(1, generalizedForces[iCoord]);
        string coordName = getModel().getCoordinateSet().get(iCoord).getName();
        const Actuator& thisAct = getActuatorSet().get(coordName);

        thisAct.addInControls(thisActuatorsControls, controls);
    }
}

void TaskSpace::Controller::connectToModel(Model& model)
{
    // Ensure that the necessary CoordinateActuator's are in the model.
    // TODO
    Super::connectToModel(model);
}
