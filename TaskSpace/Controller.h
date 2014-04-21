#ifndef OPENSIM_TASKSPACE_CONTROLLER_H_
#define OPENSIM_TASKSPACE_CONTROLLER_H_

#include <OpenSim/Simulation/Control/Controller.h>
#include "PriorityLevelSet.h"
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
 * This Controller uses CoordinateActuator's to apply the generalized forces
 * that the controller computes. The user must supply these
 * CoordinateActuator's in the model. Furthermore, the CoordinateActuator's
 * must be named as the name of the Coordinate it controls. For example, the
 * CoordinateActuator that controls the 'hip_r_flexion' Coordinate must be
 * named 'hip_r_flexion'.
 *
 * ### How it works
 *
 * A user defines a number of Task's and arranges them into PriorityLevel's.
 * This Controller then attempts to achieve the tasks in the order of
 * PriorityLevel's.
 *
 * We use Khatib's notation [1]:
 *  -\f$ n \f$: Number of coordinates (degrees of freedom) in the system.
 *  -\f$ \Gamma \in \mathbf{R}^n\f$: The output of the controller, which is the
 *      generalized forces we apply to the system.
 *  -\f$ P \f$: number of task priority levels.
 *  -\f$ p \in \{0, 1, \cdots, P - 1\}\f$: Index through task priority levels.
 *  \f$ p = 0 \f$ is the highest priority level; \f$ p = P - 1 \f$ is the
 *      lowest priority level.
 *  -\f$ \Gamma_p \in \mathbf{R}^n \f$: The generalized forces whose aim is to
 *      achieve the tasks in priority level \f$ p \f$. This is an attribute of
 *      a PriorityLevel.
 *  -\f$ N_p \in \mathbf{R}^{n \times n} \f$: Nullspace projection matrix that
 *      ensures tasks of priority lower than \f$p\f$ (that is, higher index) do not
 *      interfere with tasks of priority level \f$p\f$. This is an attribute of
 *      a PriorityLevel.
 *
 * The controls are the generalized forces that each CoordinateActuator
 * applies. For three priority levels, this controls vector is computed as:
 *
 * \f[
 *      \Gamma = \Gamma_0 + N_0^T ( \Gamma_1 + N_1^T \Gamma_2)
 * \f]
 *
 * We completely apply the generalized forces that satisfy the Task's in
 * PriorityLevel 0 (\f$ \Gamma_0 \f$).  We compute the generalized forces that
 * attempt to satisfy the Task's in PriorityLevel 1 and 2 (\f$ \Gamma_1 + N_1^T
 * \Gamma_2\f$). To ensure that these generalized forces do not interfere with
 * PriorityLevel 0 Task's, we multiply this vector of generalized forces by \f$
 * N_0 \f$, the nullspace projection matrix of PriorityLevel 0. We also do the
 * same the same to ensure that PriorityLevel 2 Task's do not interfere with
 * PriorityLevel 1 Task's.
 *
 * A PriorityLevel need only provide its \f$ \Gamma_p \f$ and its \f$ N_p \f$.
 *
 * ### Example API usage
 *
 * Here's an example of how one could create a TaskSpace::Controller:
 *
 * \code{.cpp}
 *   // Construct the Model and the Controller.
 *   Model model("my_redundant_system.osim");
 *   TaskSpace::Controller * controller = new TaskSpace::Controller();
 *   controller->setName("goal_position");
 *
 *   // Make sure the Controller has the necessary CoordinateActuator's.
 *   controller->setActuators(model.updActuators());
 *
 *   // Define the goal position task.
 *   GoalPositionTask * goalPosition;
 *   goalPosition->set_body_name("link3");
 *   goalPosition->set_position_on_body(Vec3(1.0, 0.0, 0.0));
 *
 *   // Create the 0-th PriorityLevel.
 *   PriorityLevel * level0 = new PriorityLevel();
 *   controller->updPriorityLevelSet().adoptAndAppend(level0);
 *
 *   // Add our Task to this PriorityLevel.
 *   level->updTaskSet().adoptAndAppend(goalPosition);
 *
 *   // Add our Controller to the model.
 *   model.addController(controller);
 * \endcode
 *
 * ### Limitations
 *
 * As of now, this controller works only with CoordinateActuators, and there
 * must be one such Actuator for each joint TODO.
 *
 * ### References
 *
 * [1] Khatib, Oussama, et al. "Robotics-based synthesis of human motion."
 * Journal of physiology-Paris 103.3 (2009): 211-219.
 *
 */
class OSIMTASKSPACECONTROL_API Controller : public OpenSim::Controller
{
OpenSim_DECLARE_CONCRETE_OBJECT(Controller, OpenSim::Controller);

public:

    OpenSim_DECLARE_PROPERTY(priority_levels, TaskSpace::PriorityLevelSet,
            "The PriorityLevel's that define this Controller. The priority of "
            "the levels is given by their index in the set. The first item "
            "in the set is the highest priority.");

    Controller() { }

    virtual void computeControls(const SimTK::State& s,
            SimTK::Vector& controls) const;

protected:

    void connectToModel(Model& model) OVERRIDE_11;

};

} // namespace TaskSpace

} // namespace OpenSim

#endif
