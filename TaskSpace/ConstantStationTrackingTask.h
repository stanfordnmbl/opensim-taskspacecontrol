#ifndef OPENSIM_TASKSPACE_CONSTANTSTATIONTRACKINGTASK_H_
#define OPENSIM_TASKSPACE_CONSTANTSTATIONTRACKINGTASK_H_

namespace OpenSim {

namespace TaskSpace {

/**
 * TODO
 */
class OSIMTASKSPACE_API ConstantStationTrackingTask : public TaskSpace::Task
{
OpenSim_DECLARE_CONCRETE_OBJECT(ConstantStationTrackingTask, TaskSpace::Task);
public:

    /** @name Property declarations */
    /**@{**/
    OpenSim_DECLARE_PROPERTY(desired_location, Vec3,
            "The desired location of the station, expressed in ground.");
    /**@}**/


    // -------------------------------------------------------------------------
    // Implementation of StationTrackingTask interface
    // -------------------------------------------------------------------------

    Vec3 desiredLocation(const State& s) const OVERRIDE_11 final
    {
        return get_desired_location();
    }

    virtual Vec3 desiredVelocity(const State& s) const OVERRIDE_11 final
    {
        return Vec3(0);
    }
    
    virtual Vec3 desiredAcceleration(const State& s) const OVERRIDE_11 final
    {
        return Vec3(0);
    }

};

} // namespace TaskSpace

} // namespace OpenSim

#endif
