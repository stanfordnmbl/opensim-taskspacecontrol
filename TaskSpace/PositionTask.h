#ifndef OPENSIM_TASKSPACE_POSITIONTASK_H_
#define OPENSIM_TASKSPACE_POSITIONTASK_H_

namespace OpenSim {

namespace TaskSpace {

/** @brief A task that deals with the desired behavior of a point that is fixed
 * on one of the bodies in the system.
 *
 * The simplest such task would be to drive
 * this station to be coincident with a point in the ground frame. This point
 * in the ground frame could be a marker trajectory. The task could be that, at
 * this point, the system applies a certain force on its environment.
 *
 * TODO rename as StationTask.
 *
 */
class OSIMTASKSPACE_API PositionTask : public TaskSpace::Task
{
OpenSim_DECLARE_CONCRETE_OBJECT(PositionTask, TaskSpace::Task);
public:

    /** @name Property declarations */
    /**@{**/
    // goalPosition->set_body_name("link3");
    // goalPosition->set_position_on_body(Vec3(1.0, 0.0, 0.0));
    // TODO compensateForQuadraticVelocity
    // TODO compensateForGravity
    /**@}**/

    /** Expressed in ground.
     * */
    Vector taskSpaceForce(State& s) const;

    Matrix nullspaceProjectionTranspose(const State& s) const;

    Vector generalizedForces(const State& s) const;

    Matrix taskSpaceMassMatrix(const State& s) const;
    
    Matrix jacobianGeneralizedInverse(const State& s) const;

    Matrix taskSpaceQuadraticVelocity(const State& s) const;

    Matrix taskSpaceGravity(const State& s) const;

    /** Like an acceleration.
     * */
    // OVERRIDE_11
    virtual Vector controlLaw(const State& s) const = 0;

};

} // namespace TaskSpace

} // namespace OpenSim

#endif
