#ifndef OPENSIM_TASKSPACE_PRIORITYLEVEL_H_
#define OPENSIM_TASKSPACE_PRIORITYLEVEL_H_

using SimTK::State;
using SimTK::Matrix;
using SimTK::Vector;

namespace OpenSim {

namespace TaskSpace {

class OSIMTASKSPACE_API PriorityLevel : public OpenSim::Object
{
OpenSim_DECLARE_CONCRETE_OBJECT(PriorityLevel, OpenSim::Object);
public:
    /** @name Property declarations */
    /**@{**/
    OpenSim_DECLARE_PROPERTY(tasks, TaskSpace::TaskSet,
            "All the tasks in this priority level.");
    /**@}**/

public:
    Matrix nullspaceProjectionTranspose(const State& s);
    Vector generalizedForce(const State& s);

} // namespace TaskSpace

} // namespace OpenSim

#endif
