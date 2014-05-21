#ifndef OPENSIM_TASKSPACE_GRAVITYCOMPENSATIONTASK_H_
#define OPENSIM_TASKSPACE_GRAVITYCOMPENSATIONTASK_H_

#include <OpenSim/Simulation/Model/Model.h>

#include "Task.h"

#include "osimTaskSpaceControlDLL.h"

namespace OpenSim {

namespace TaskSpace {

/**
 * TODO
 */
class OSIMTASKSPACECONTROL_API GravityCompensationTask : public TaskSpace::Task
{
OpenSim_DECLARE_CONCRETE_OBJECT(TaskSpace::GravityCompensationTask,
        TaskSpace::Task);
public:


    // -------------------------------------------------------------------------
    // Implementation of Task interface
    // -------------------------------------------------------------------------

    unsigned int getNumScalarTasks() const OVERRIDE_11 final
    {
        // TODO replace with state.getNU().
        return m_model->getCoordinateSet().getSize();
    }

    Matrix jacobian(const State& s) const OVERRIDE_11 final
    {
        // TODO can be created at compile-time? maybe not, if constraints are
        // being added/removed.
        Matrix jac(s.getNU(), s.getNU());
        jac.setToZero();
        jac.diag().setTo(1.0);
        return jac;
    }

    Vector taskSpaceForces(const State& s) const OVERRIDE_11 final
    {
        return -taskSpaceGravity(s);
    }

    // TODO documentation?
    Vector taskSpaceQuadraticVelocity(const State& s) const OVERRIDE_11 final
    {
        throw Exception("Unimplemented.");
    }

};

} // namespace TaskSpace

} // namespace OpenSim

#endif
