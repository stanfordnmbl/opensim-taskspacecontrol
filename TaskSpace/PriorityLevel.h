#ifndef OPENSIM_TASKSPACE_PRIORITYLEVEL_H_
#define OPENSIM_TASKSPACE_PRIORITYLEVEL_H_

#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Property.h>
#include "TaskSet.h"
#include "osimTaskSpaceControlDLL.h"

using SimTK::SimbodyMatterSubsystem;
using SimTK::State;
using SimTK::Matrix;
using SimTK::Vector;

namespace OpenSim {

class Model;

namespace TaskSpace {

class Controller;

/**
 * A PriorityLevel, identified by its index \f$ p \f$, must be able to compute
 * two quantities (\f$ n \f$ is the number of coordinates in the model):
 * 1. generalized forces
 * 2. a nullspace projection matrix
 *
 * The nullspace projection matrix depends on:
 * - this PriorityLevel's jacobian, obtained from its constituent Task's.
 * - the system's mass matrix, computed by Simbody.
 * - this PriorityLevel's task-space mass matrix.
 *
 * The task-space mass matrix depends on:
 * - the system's mass matrix
 * - this PriorityLevel's jacobian
 *
 * See the corresponding methods for more information.
 *
 */
class OSIMTASKSPACECONTROL_API PriorityLevel : public OpenSim::Object
{
OpenSim_DECLARE_CONCRETE_OBJECT(TaskSpace::PriorityLevel, OpenSim::Object);

public:
    /** @name Property declarations */
    /**@{**/
    OpenSim_DECLARE_PROPERTY(tasks, TaskSpace::TaskSet,
            "All the tasks in this priority level.");
    /**@}**/


    PriorityLevel();


    // -------------------------------------------------------------------------
    // Methods used by the TaskSpace::Controller
    // -------------------------------------------------------------------------

    /**
     * @brief This quantity is denoted as \f$ \Gamma_p \in \mathbf{R}^n \f$.
     * These generalized forces attempt to achieve all Task's within the
     * PriorityLevel.
     *
     * This Vector is obtained by concatenating the generalized forces of the
     * constituent Task:
     *
     * \f[
     * \Gamma_p =
     * \begin{bmatrix}
     *       \Gamma_{p1} \\
     *       \Gamma_{p2} \\
     *       \vdots \\
     *       \Gamma_{pT}
     * \end{bmatrix}
     * \f]
     *
     * where \f$ T \f$ is the number of Task's in this PriorityLevel, and
     * \f$ \Gamma_{pt} \f$ is the generalized forces for the \f$t\f$-th Task in
     * PriorityLevel \f$p\f$.
     */
    Vector generalizedForces(const State& s);

    /**
     * @brief This quantity is denoted as \f$ N_p \in \mathbf{R}^{n \times n}
     * \f$, and is used to ensure tasks of priority lower than this one do not
     * interfere with tasks of this PriorityLevel.
     *
     * This Matrix is computed via:
     *
     * \f[
     * N_p = I - \bar{J}_p J_p
     * \f]
     *
     * where:
     * - \f$ I \in \mathbf{R}^{n \times n} \f$ is the identity matrix.
     * - \f$ \bar{J}_p \f$ is this PriorityLevel's dynamically-consistent
     *   jacobian inverse (see related method).
     * - \f$ J_p \f$ is this PriorityLevel's jacobian (see related method).
     *
     */
    Matrix nullspaceProjection(const State& s);


    // -------------------------------------------------------------------------
    // Member functions
    // -------------------------------------------------------------------------

    /**
     * TODO
     */
    unsigned int getNumScalarTasks() const
    {
        return m_numScalarTasks;
    }

    /**
     * @brief This quantity is denoted as \f$ J_p \in \mathbf{R}^{S \times
     * n}\f$, where \f$S\f$ is the number of scalar tasks in this priority
     * level, and \f$n\f$ is the number of degrees of freedom of the Model.
     *
     * This matrix is most often used to convert generalized speeds into
     * Cartesian (task-space) velocities, or Cartesian (task-space) forces into
     * generalized forces.
     *
     * This Matrix is obtained by concatenating the jacobians of the
     * constituent Task's:
     *
     * \f[
     * J_p =
     * \begin{bmatrix}
     *      J_{p1} \\
     *      J_{p2} \\
     *      \vdots \\
     *      J_{pT}
     * \end{bmatrix}
     *
     * \f]
     */
    Matrix jacobian(const State& s);

    /**
     * @brief This quantity is denoted as \f$ \bar{J}_p \in \mathbf{R}^{n
     * \times S} \f$, where \f$n\f$ is the number of degrees of freedom of the
     * Model, and \f$S\f$ is the number of scalar tasks in this PriorityLevel.
     *
     * This Matrix is computed via:
     *
     * \f[
     * \bar{J}_p = A^{-1} J_p^T \Lambda_p
     * \f]
     *
     * where:
     * - \f$ A \in \mathbf{R}^{n \times n} \f$ is the system's mass matrix (in
     *   generalized coordinates).
     * - \f$ J_p \f$ is this PriorityLevel's jacobian (see related method).
     * - \f$ \Lambda_p \f$ is this PriorityLevel's task-space mass matrix (see
     *   related method).
     *
     */
    Matrix dynamicallyConsistentJacobianInverse(const State& s);

    /**
     * @brief This quantity is denoted as \f$ \Lambda_p \in \mathbf{R}^{S
     * \times S} \f$, where \f$S\f$ is the number of scalar tasks in this
     * PriorityLevel.
     *
     * Ths Matrix is computed via:
     *
     * \f[
     * \Lambda_p = (J_p A^{-1} J_p^T)^{-1}
     * \f]
     *
     * where:
     * - \f$ A \in \mathbf{R}^{n \times n} \f$ is the system's mass matrix (in
     *   the \f$ n \f$ generalized coordinates).
     * - \f$ J_p \f$ is this PriorityLevel's jacobian (see related method).
     */
    Matrix taskSpaceMassMatrix(const State& s);

private:

    void setModel(const Model& model);

    const Model* m_model;

    unsigned int m_numScalarTasks;

    friend class TaskSpace::Controller;

    void setNull();
    void constructProperties();

};

} // namespace TaskSpace

} // namespace OpenSim

#endif
