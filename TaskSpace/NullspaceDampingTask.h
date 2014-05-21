#ifndef OPENSIM_TASKSPACE_NULLSPACEDAMPINGTASK_H_
#define OPENSIM_TASKSPACE_NULLSPACEDAMPINGTASK_H_

#include <OpenSim/Simulation/Model/Model.h>

#include "Task.h"

#include "osimTaskSpaceControlDLL.h"

using SimTK::SimbodyMatterSubsystem;
using SimTK::State;
using SimTK::Matrix;
using SimTK::Vector;

namespace OpenSim {

namespace TaskSpace {

/**
 * @brief TODO
 *
 * TODO
 */
class OSIMTASKSPACECONTROL_API NullspaceDampingTask : public TaskSpace::Task
{
OpenSim_DECLARE_ABSTRACT_OBJECT(TaskSpace::NullspaceDampingTask,
        TaskSpace::Task);

public:

    NullspaceDampingTask()
    {
        setNull();
    }


    // -------------------------------------------------------------------------
    // Specification of Task interface; methods used by TaskSpace::PriorityLevel
    // -------------------------------------------------------------------------

    /**
     * @brief Denoted as \f$ S \f$ throughout the documentation.
     *
     * The jacobian describes the "location" of the constituent scalar tasks.
     * It allows one to convert generalized speeds into Cartesian (task-space)
     * velocities, and Cartesian (task-space) forces into generalized forces.
     *
     * This is always constant in time (or, known at compile-time). For some
     * tasks, this is static (the same for all instances of the Task). For some
     * Task's, however, this number could depend on the specific definition of
     * the Task, or on the specific model being controlled.
     */
    unsigned int getNumScalarTasks(const State& s) const OVERRIDE_11 final
    {
        return s.getNU();
    }

    Matrix jacobian(const State& s) const OVERRIDE_11 final
    {
        // TODO sparse representation?
        Matrix jac(s.getNU(), s.getNU());
        jac.setToZero();
        jac.diag(0.setTo(1.0));
        return jac;
    }

    Vector taskSpaceForces(const State& s) const OVERRIDE_11 final
    {
        Vector jac
        return 
    }

    Vector taskSpaceQuadraticVelocity(const State& s) const OVERRIDE_11 final
    {
    }

};

} // namespace TaskSpace

} // namespace OpenSim
