#ifndef OPENSIM_TASKSPACE_PRIORITYLEVELSET_H_
#define OPENSIM_TASKSPACE_PRIORITYLEVELSET_H_

#include <OpenSim/Common/Set.h>
#include "osimTaskSpaceControlDLL.h"

namespace OpenSim {

namespace TaskSpace {

class OSIMTASKSPACE_API PriorityLevelSet : public Set<PriorityLevelSet>
{
OpenSim_DECLARE_CONCRETE_CLASS(PriorityLevelSet, Set<PriorityLevelSet>);

};

} // namespace TaskSpace

} // namespace OpenSim

#endif
