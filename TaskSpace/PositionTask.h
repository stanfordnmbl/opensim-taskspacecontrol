#ifndef OPENSIM_TASKSPACE_POSITIONTASK_H_
#define OPENSIM_TASKSPACE_POSITIONTASK_H_

namespace OpenSim {

namespace TaskSpace {

class OSIMTASKSPACE_API PositionTask : public TaskSpace::Task
{
OpenSim_DECLARE_CONCRETE_OBJECT(PositionTask, TaskSpace::Task);
public:

    /** @name Property declarations */
    /**@{**/
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

} // namespace TaskSpace

} // namespace OpenSim

#endif
