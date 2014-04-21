#ifndef OPENSIM_TASKSPACE_TASK_H_
#define OPENSIM_TASKSPACE_TASK_H_

using SimTK::State;
using SimTK::Matrix;
using SimTK::Vector;

/* TODO
 * A gc task is quite different....can only be at the end? can't be in the
 * middle of some nullspace.
 * Ensure that we can have tasks that are force-control.
 * */
namespace OpenSim {

namespace TaskSpace {

/**
 * @brief This object contains a number of scalar tasks (total of \f$ S \f$).
 *
 * A scalar task is related to a single Cartesian (task-space) direction fixed
 * in a body, or a single degree of freedom. See subclasses for possibilities.
 *
 * The jacobian describes the "location" of the constituent scalar tasks, while
 * the generalized forces detail how the task is achieved.
 *
 */
class OSIMTASKSPACE_API Task : public OpenSim::Object
{
OpenSim_DECLARE_ABSTRACT_OBJECT(Task, OpenSim::Object);

public:

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
    virtual unsigned int getNumScalarTasks() const = 0;

    /**
     * @brief This quantity is denoted as \f$ J_{pt} \in \mathbf{R}^{s} \f$.
     */
    virtual Matrix jacobian(const State& s) const = 0;

    /**
     * @brief This quantity is denoted as \f$ \Gamma_{pt} \in \mathbf{R}^{s}
     * \f$.
     *
     * The generalized forces are designed to achieve the related scalar tasks.
     */
    virtual Vector generalizedForces(const State& s) const = 0;

};

} // namespace TaskSpace

} // namespace OpenSim

#endif
