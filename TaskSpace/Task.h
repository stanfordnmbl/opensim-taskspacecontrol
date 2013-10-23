#ifndef OPENSIM_TASKSPACE_TASK_H_
#define OPENSIM_TASKSPACE_TASK_H_

using SimTK::State;
using SimTK::Matrix;
using SimTK::Vector;

/* TODO
 * Maybe there is no PriorityLevel; there's just tasks, in an order, and you
 * can have a Composite task that has as many other tasks as you'd like. What
 * do we want the interface to be? specify a priority level as an attribute of
 * each task, or add tasks to a PriorityLevel object?
 * Tasks should be able to be as many dimensions as they want to be.
 * A gc task is quite different....can only be at the end? can't be in the
 * middle of some nullspace.
 * Ensure that we can have tasks that are force-control.
 * */
namespace OpenSim {

namespace TaskSpace {

class OSIMTASKSPACE_API Task : public OpenSim::Object
{
OpenSim_DECLARE_ABSTRACT_OBJECT(Task, OpenSim::Object);

public:
    /** @name Property declarations */
    /**@{**/
    /**@}**/

    virtual Matrix nullspaceProjectionTranspose(const State& s) const = 0;

    // TODO OVERRIDE_11
    virtual Vector generalizedForces(const State& s) const = 0;

};

} // namespace TaskSpace

} // namespace OpenSim

#endif
