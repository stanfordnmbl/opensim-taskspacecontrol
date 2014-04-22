#ifndef OPENSIM_TASKSPACE_TASKSET_H_
#define OPENSIM_TASKSPACE_TASKSET_H_

#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Set.h>
#include "Task.h"
#include "osimTaskSpaceControlDLL.h"

namespace OpenSim {

namespace TaskSpace {

class OSIMTASKSPACECONTROL_API TaskSet : public OpenSim::Set<Task>
{
OpenSim_DECLARE_CONCRETE_OBJECT(TaskSet, OpenSim::Set<Task>);

public:
    TaskSet() : OpenSim::Set<Task>() { }
    
    TaskSet(const TaskSet& aTaskSet) : OpenSim::Set<Task>(aTaskSet)
    {
        *this = aTaskSet;
    }

};

} // namespace TaskSpace

} // namespace OpenSim

#endif
