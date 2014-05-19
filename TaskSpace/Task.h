#ifndef OPENSIM_TASKSPACE_TASK_H_
#define OPENSIM_TASKSPACE_TASK_H_

#include <OpenSim/Simulation/Model/Model.h>
#include "osimTaskSpaceControlDLL.h"

using SimTK::SimbodyMatterSubsystem;
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

class PriorityLevel;

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
class OSIMTASKSPACECONTROL_API Task : public OpenSim::Object
{
OpenSim_DECLARE_ABSTRACT_OBJECT(TaskSpace::Task, OpenSim::Object);

public:

    Task()
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
    virtual unsigned int getNumScalarTasks() const = 0;

    /**
     * @brief This quantity is denoted as \f$ J_{pt} \in \mathbf{R}^{S} \f$.
     */
    virtual Matrix jacobian(const State& s) const = 0;

    /**
     * @brief This quantity is denoted as \f$ F_{pt} \in \mathbf{R}^{S}
     * \f$.
     *
     * The task-space forces are designed to achieve the related scalar tasks.
     */
    virtual Vector taskSpaceForces(const State& s) const = 0;

    /**
     * @brief \f$ \mu_{pt} = \bar{J}^T b - \Lambda_{pt} \dot{J}_{pt} \dot{q}
     * \in \mathbf{R}^3 \f$.  
     *
     * where:
     * - \f$ b \f$ is the quadratic velocity vector for the whole system, in
     *   generalized coordinates.
     * - \f$ \Lambda_{pt} \f$ is the task space mass matrix of this task.
     * - \f$ \dot{J}_{pt} \f$ is the derivative of this Tasks' jacobian.
     * - \f$ q \f$ is the generalized coordinates.
     *
     * Used in taskSpaceForce(). In Simbody, this cannot be computed as a
     * function of the jacobian(s). Therefore, the specific Task must provide
     * this.
     *
     */
    virtual Vector taskSpaceQuadraticVelocity(const State& s) const = 0;


    // -------------------------------------------------------------------------
    // Member functions
    // -------------------------------------------------------------------------
    // TODO the existence of (most of) these methods in Task indicates that
    // PriorityLevel should just be a CompositeTask.

    /**
     * @brief This quantity is denoted as \f$ \Gamma_{pt} \in \mathbf{R}^{S}
     * \f$.
     *
     * The generalized forces are obtained via:
     *
     * \f[
     *      \Gamma_{pt} = J_{pt}^T F_{pt}
     * \f]
     *
     */
    virtual Vector generalizedForces(const State& s) const;

    /**
     * @brief This quantity is denoted as \f$ \bar{J}_{pt} \in \mathbf{R}^{n
     * \times S} \f$, where \f$n\f$ is the number of degrees of freedom of the
     * Model, and \f$S\f$ is the number of scalar tasks in this Task.
     *
     * This Matrix is computed via:
     *
     * \f[
     * \bar{J}_{pt} = A^{-1} J_{pt}^T \Lambda_{pt}
     * \f]
     *
     * where:
     * - \f$ A \in \mathbf{R}^{n \times n} \f$ is the system's mass matrix (in
     *   generalized coordinates).
     * - \f$ J_{pt} \f$ is this Task's jacobian (see related method).
     * - \f$ \Lambda_{pt} \f$ is this Task's task-space mass matrix (see
     *   related method).
     *
     */
    Matrix dynamicallyConsistentJacobianInverse(const State& s) const;

    /**
     * @brief This quantity is denoted as \f$ \Lambda_{pt} \in \mathbf{R}^{S
     * \times S} \f$, where \f$S\f$ is the number of scalar tasks in this Task.
     *
     * Ths Matrix is computed via:
     *
     * \f[
     * \Lambda_{pt} = (J_{pt} A^{-1} J_{pt}^T)^{-1}
     * \f]
     *
     * where:
     * - \f$ A \in \mathbf{R}^{n \times n} \f$ is the system's mass matrix (in
     *   the \f$ n \f$ generalized coordinates).
     * - \f$ J_{pt} \f$ is this Task's jacobian (see related method).
     */
    Matrix taskSpaceMassMatrix(const State& s) const;

    /**
     * @brief \f$ p_{pt} = \bar{J}^T g \in \mathbf{R}^3 \f$.
     *
     * where \f$g\f$ is the gravity vector for the whole system, in generalized
     * coordinates.
     *
     * Used in taskSpaceForce().
     */
    Vector taskSpaceGravity(const State& s) const;

protected:

    virtual void setModel(const Model& model)
    {
        m_model = &model;
    }

    const Model* m_model;

private:

    // TODO replace with connectors.
    friend class PriorityLevel;

    void setNull()
    {
        m_model = NULL;
    }

};

} // namespace TaskSpace

} // namespace OpenSim

#endif
