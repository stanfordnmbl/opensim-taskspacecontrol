#ifndef OPENSIM_TASKSPACE_TASK_H_
#define OPENSIM_TASKSPACE_TASK_H_

#include <OpenSim/Simulation/Model/Model.h>
#include "PriorityLevel.h"
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
OpenSim_DECLARE_ABSTRACT_OBJECT(Task, OpenSim::Object);

public:

    Task()
    {
        setNull();
    }

    // -------------------------------------------------------------------------
    // Specification of Task interface
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


    // -------------------------------------------------------------------------
    // Member functions
    // -------------------------------------------------------------------------

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
    Matrix dynamicallyConsistentJacobianInverse(const State& s);

    /**
     * @brief This quantity is denoted as \f$ \Lambda_{pt} \in \mathbf{R}^{S
     * \times S} \f$, where \f$S\f$ is the number of scalar tasks in this Task.
     *
     * Ths Matrix is computed via:
     *
     * \f[
     * \Lambda_p = (J_{pt} A^{-1} J_{pt}^T)^{-1}
     * \f]
     *
     * where:
     * - \f$ A \in \mathbf{R}^{n \times n} \f$ is the system's mass matrix (in
     *   the \f$ n \f$ generalized coordinates).
     * - \f$ J_{pt} \f$ is this Task's jacobian (see related method).
     */
    Matrix taskSpaceMassMatrix(const State& s);

protected:

    const Model* m_model;
    const SimbodyMatterSubsystem* m_smss;

private:

    void setModel(const Model& model)
    {
        m_model = &model;
        m_smss = &model.getMatterSubsystem();
    }

    friend class PriorityLevel;

    void setNull()
    {
        m_model = NULL;
        m_smss = NULL;
    }

};

} // namespace TaskSpace

} // namespace OpenSim

#endif
