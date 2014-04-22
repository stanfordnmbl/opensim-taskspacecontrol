#ifndef OPENSIM_TASKSPACE_STATIONTASK_H_
#define OPENSIM_TASKSPACE_STATIONTASK_H_

#include "osimTaskSpaceControlDLL.h"

#include "Task.h"

using std::string;
using SimTK::Vec3;

namespace OpenSim {

namespace TaskSpace {

/**
 * @brief A task that deals with the desired behavior of a point that is fixed
 * on one of the bodies in the system. In Simbody's terminology, such a point
 * is called a station.
 *
 * The simplest such task would be to drive this station to be coincident with
 * a point in the ground frame. This point in the ground frame could be a
 * marker trajectory. Another possible task is for the system to apply a
 * desired force on its environment at this point.
 *
 * The way one defines a StationTask is by defining a control law that is
 * designed to achieve the desired task. See the controlLaw method for more
 * information.
 *
 * It is likely that the control law depends on the position and velocity
 * of the station in the ground frame. Thus, we provide convenient methods to
 * access these quantities. It is also likely that the control law depends on
 * other quantities related to the multibody system. To compute other
 * quantities, obtain a reference to the Model's SimbodyMatterSubsystem, which
 * can be obtained from the Model.
 *
 * This class does the rest of the necessary computation to incorporate this
 * task with the rest of the task-space controller.
 *
 */
class OSIMTASKSPACECONTROL_API StationTask : public TaskSpace::Task
{
OpenSim_DECLARE_CONCRETE_OBJECT(StationTask, TaskSpace::Task);
public:

    /** @name Property declarations */
    /**@{**/
    OpenSim_DECLARE_PROPERTY(body_name, string,
            "The name of the body on which the station resides.");
    OpenSim_DECLARE_PROPERTY(location_in_body, Vec3,
            "Expressed in the body frame.");
    // TODO compensateForQuadraticVelocity
    // TODO compensateForGravity
    /**@}**/


    StationTask() { }


    // -------------------------------------------------------------------------
    // Specification of StationTask interface
    // -------------------------------------------------------------------------

    /**
     * @brief The acceleration of the station, expressed in ground, that is
     * designed to achieve the task.
     *
     * For example, the control law could be
     * a proportional-derivative control law. The control law is a 3-vector
     * expressed in the ground frame, and describes the force/acceleration,
     * expressed in the ground frame, that we want at the station. In
     * Khatib's terminology, the control law is denoted as \f$ F^{*} \f$.
     *
     */
    virtual Vec3 controlLaw(const State& s) const = 0;

    // -------------------------------------------------------------------------
    // Implementation of Task interface
    // -------------------------------------------------------------------------

    unsigned int getNumScalarTasks() const OVERRIDE_11 final { return 3; }

    /**
     * @brief \f$ \Gamma_{pt} = J_{pt}^T F_{pt} \f$.
     *
     * See the other methods in this class for \f$J_{pt}\f$ and \f$F_{pt}\f$.
     */
    Vector generalizedForces(const State& s) const OVERRIDE_11 final;

    /**
     * @brief \f$ J_{pt} \in \mathbf{R}^{3 \times n} \f$, where \f$n\f$ is the
     * number of degrees of freedom in the Model.
     *
     * This is obtained from Simbody.
     */
    Matrix jacobian(const State& s) const OVERRIDE_11 final;


    // -------------------------------------------------------------------------
    // Member functions
    // -------------------------------------------------------------------------

    /**
     * @brief \f$ F_{pt} = \Lambda_{pt} F^{*} + \mu_{pt} + p_{pt} \in
     * \mathbf{R}^3 \f$: The force at the station, expressed in ground, that
     * achieves the control law and also compensates for the dynamics of the
     * system.
     */
    Vec3 taskSpaceForce(State& s) const;

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
     * Used in taskSpaceForce().
     */
    Vec3 taskSpaceQuadraticVelocityCompensation(const State& s) const;

    /**
     * @brief \f$ p_{pt} = \bar{J}^T g \in \mathbf{R}^3 \f$.
     *
     * where \f$g\f$ is the gravity vector for the whole system, in generalized
     * coordinates.
     *
     * Used in taskSpaceForce().
     */
    Vec3 taskSpaceGravityCompensation(const State& s) const;


    // -------------------------------------------------------------------------
    // Convenient methods for subclasses, for defining control laws.
    // -------------------------------------------------------------------------

    /**
     * @brief Location of the station, expressed in the ground frame.
     */
    Vec3 locationOfStationExpressedInGround(const State& s) const
    {
        return m_smss->findStationLocationInGround(s, get_location_in_body());
    }

    /**
     * @brief Velocity of the station relative to the ground frame, expressed
     * in the ground frame.
     */
    Vec3 velocityOfStationInGroundExpressedInGround(const State& s) const
    {
        return m_smss->findStationVelocityInGround(s, get_location_in_body());
    }

};

} // namespace TaskSpace

} // namespace OpenSim

#endif
