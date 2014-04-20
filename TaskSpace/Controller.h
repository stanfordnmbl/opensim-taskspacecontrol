#ifndef OPENSIM_TASKSPACE_CONTROLLER_H_
#define OPENSIM_TASKSPACE_CONTROLLER_H_

#include <OpenSim/Simulation/Control/Controller.h>
#include "osimTaskSpaceControlDLL.h"

namespace OpenSim {

namespace TaskSpace {

/**
 * This Controller determines the generalized forces to apply to a Model's
 * mobilities so that the model achieve specified tasks. The tasks can be
 * specified in a hierarchy: we only attempt to satisfy
 * lower-priority tasks after we have satisfied higher-priority tasks. For
 * example, we may have a high priority task that a robot arm avoids an
 * obstacle in its workspace. We may have a second lower priority task that the
 * robot's end effector is at a desired position.
 *
 * ### How it works
 *
 * A user defines a number of Task's and arranges them into PriorityLevel's.
 * This Controller then attempts to achieve the tasks in the order of
 * PriorityLevel's.
 *
 * We use Khatib's notation:
 *  -\f$ n \f$: number of coordinates (degrees of freedom) in the system.
 *  -\f$ \Gamma \in \mathbf{R}^n\f$: The output of the controller, which is the
 *      generalized forces we apply to the system.
 *  -\f$ P \f$: number of task priority levels.
 *  -\f$ p \in \{0, 1, \cdots, P - 1\}\f$: index through task priority levels.
 *  -\f$ \Gamma_i \in \mathbf{R}^n \f$: the generalized forces whose aim is to
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
 * A specific task-space controller is defined via TaskSpace::PriorityLevel's
 * and TaskSpace::Task's. TODO describe organization of classes.
 *
 * A PriorityLevel need only provide its \f$ \Gamma_p \f$ and its \f$ N_p \f$.
 *
 *
 * ### Example API usage
 *
 * Here's an example of how you may use
 * TODO
 *
 * ### Limitations
 *
 * As of now, this controller
 * works only with CoordinateActuators, and there must be one for each joint
 * TODO.
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
