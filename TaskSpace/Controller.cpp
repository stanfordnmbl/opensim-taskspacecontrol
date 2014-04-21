#include "Controller.h"

using SimTK::Matrix;
using SimTK::State;
using SimTK::Vector;

using namespace OpenSim;

void TaskSpace::Controller::computeControls(const State& s,
        Vector& controls) const
{
    // The control vector.
    // -------------------
    
    // Gamma = Gamma_0 + N_0^T * (Gamma_1 + N_1^T * (Gamma_2 + N_2^T * (...)))
    // We compute the inner parentheses first. This is like how it's more
    // efficient to compute the polynomial a + bx + cx^2 + dx^3 as:
    //      a + x(b + x(c + xd))
    Vector generalizedForces(_numCoords);
    generalizedForces.setToZero();

    for (unsigned int iP = get_priority_levels().getSize() - 1; iP > 0; iP--)
    {
        Matrix NT =
            get_priority_levels().get(iP - 1).nullspaceProjection(s).transpose();
        Vector Gamma_iP = get_priority_levels().get(iP).generalizedForce(s);

        generalizedForces = NT * (Gamma_iP + generalizedForces);
    }

    // The highest-priority level doesn't get filtered by a nullspace
    // projection.
    generalizedForces =
        get_priority_levels().get(0).generalizedForce() + generalizedForces;

    // Send control signals to CoordinateActuator's.
    // ---------------------------------------------
    // TODO what if a coordinate is disabled? etc. Many issues to consider here.
    // What if an actuator is disabled?
    // What if there is no actuator with name coordName?
    // What if the actuator with coordName is not a CoordinateActuator?
    // What if there are quaternions (num coords != num speeds)?
    for (unsigned int iCoord = 0;
            iCoord < getModel().getCoordinateSet().getSize(); iCoord++)
    {
        Vector thisActuatorsControls(1, generalizedForces[iCoord]);
        string coordName = getModel().getCoordinateSet().get(iCoord).getName();
        const Actuator& thisAct = getActuatorSet().get(coordName);

        thisAct.addInControls(thisActuatorsControls, controls);
    }
}

void TaskSpace::Controller::connectToModel(Model& model)
{
    Super::connectToModel(model);

    // Ensure that the necessary CoordinateActuator's are in the model.
    // TODO

    for (unsigned int iP = 0; iP < get_priority_levels().getSize(); iP++)
    {
        get_priority_levels().upd(iP).setModel(model);
    }
}
