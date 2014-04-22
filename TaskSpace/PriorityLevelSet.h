#ifndef OPENSIM_TASKSPACE_PRIORITYLEVELSET_H_
#define OPENSIM_TASKSPACE_PRIORITYLEVELSET_H_

#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Set.h>
#include "PriorityLevel.h"
#include "osimTaskSpaceControlDLL.h"

namespace OpenSim {

namespace TaskSpace {

class OSIMTASKSPACECONTROL_API PriorityLevelSet : public OpenSim::Set<PriorityLevel>
{
OpenSim_DECLARE_CONCRETE_OBJECT(PriorityLevelSet, OpenSim::Set<PriorityLevel>);

public:
    PriorityLevelSet() : OpenSim::Set<PriorityLevel>() { }

    PriorityLevelSet(const PriorityLevelSet& aPriorityLevelSet) :
        OpenSim::Set<PriorityLevel>(aPriorityLevelSet)
    {
        *this = aPriorityLevelSet;
    }

};

} // namespace TaskSpace

} // namespace OpenSim

#endif
