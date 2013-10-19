#ifndef OPENSIM_TASKSPACE_POSITIONTASK_H_
#define OPENSIM_TASKSPACE_POSITIONTASK_H_

namespace OpenSim {

namespace TaskSpace {

class OSIMTASKSPACE_API PositionTask : public TaskSpace::Task
{
OpenSim_DECLARE_CONCRETE_OBJECT(PositionTask, TaskSpace::Task);
public:

    virtual Vec3 desiredPosition(const State& s) const = 0;
    virtual Vec3 desiredVelocity(const State& s) const = 0;
    virtual Vec3 desiredAcceleration(const State& s) const = 0;

    Vec3 positionInGround(const State& s) const;
    Vec3 velocityInGround(const State& s) const;

    /** Like an acceleration.
     * */
    virtual Vector controlLaw(const State& s) const;


} // namespace TaskSpace

} // namespace OpenSim

#endif
