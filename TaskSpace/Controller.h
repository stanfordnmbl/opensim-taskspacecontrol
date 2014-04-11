#ifndef OPENSIM_TASKSPACE_CONTROLLER_H_
#define OPENSIM_TASKSPACE_CONTROLLER_H_

#include <OpenSim/Simulation/Control/Controller.h>
#include "osimTaskSpaceControlDLL.h"

namespace OpenSim {

namespace TaskSpace {

/**
 * This Controller determines the generalized forces that achieve specified
 * tasks. The tasks can be specified within a hierarchy, so that tasks are
 * achieved in an order. TODO wording. As of now, this controller works only
 * with CoordinateActuators, and there must be one for each joint.
 *
 * We use Khatib's notation:
 *  -\f$ N \f$: number of coordinates (degrees of freedom) in the system.
 *  -\f$ \Gamma \in \mathbf{R}^N\f$: The output of the controller, which is the
 *      generalized forces we apply, for the entire system. 
 *  -\f$ p \f$: index through task priority levels.
 *  -\f$ P \f$: number of task priority levels.
 *  -\f$ \Gamma_i \in \mathbf{R}^N \f$: the generalized forces whose aim is to
 *      achieve the tasks in priority level \f$ i \f$. If the task is specified
 *      in the space of the generalized coordinates, then the task computes
 *      this directly. If the task is a task-space task, meaning it is
 *      specified in Cartesian coordinates, then this is computed with the help
 *      of the task's jacobian TODO.
 *
 * The controls are the generalized forces that each CoordinateActuator
 * applies. For 3 priority levels, this controls vector is computed as:
 *
 * \f[
 *      \Gamma = \Gamma_0 + N_0^T ( \Gamma_1 + N_1^T \Gamma_2)
 * \f]
 * (adapted from course notes from CS 327 at Stanford)
 *
 * TODO interpretation.
 *
 * This can be computed recursively. We show the procedure for \f$P\f$ priority
 * levels in pseudocode.
 *
 * \f{eqnarray*}{
 *      \Gamma &\gets& \Gamma_P \\
 *      \Gamma &\gets& \Gamma_{p} + N_{p} \Gamma
 * \f}
 * TODO for p = P-1...0
 *
 * For P tasks, the formula above generalizes to:
 *
 * \f[
 *      \Gamma = \Gamma_0 + N_{0}^T \Gamma_1 + N_0^T N_1^T \Gamma_2 \cdots
 * \f]
 *
 *
 */
class OSIMTASKSPACECONTROL_API Controller : public OpenSim::Controller
{
OpenSim_DECLARE_CONCRETE_OBJECT(Controller, OpenSim::Controller);

public:
    virtual void computeControls(const SimTK::State& s,
            SimTK::Vector& controls) const;

protected:
    void connectToModel(Model& model) OVERRIDE_11;

};

} // namespace TaskSpace

} // namespace OpenSim

#endif
