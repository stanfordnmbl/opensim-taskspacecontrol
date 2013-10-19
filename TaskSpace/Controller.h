#ifndef OPENSIM_TASKSPACE_CONTROLLER_H_
#define OPENSIM_TASKSPACE_CONTROLLER_H_

#include "SimTKsimbody.h"

namespace OpenSim {

namespace TaskSpace {

class OSIMTASKSPACE_API Controller : public OpenSim::Controller
{
OpenSim_DECLARE_CONCRETE_OBJECT(Controller, OpenSim::Controller);

public:
    virtual void computeControls(const SimTK::State& s,
            SimTK::Vector& controls) const;

protected:
    void connectToModel(Model& model) OVERRIDE_11;

};

} // namespace TaskSpace

} // namespace OpenSim

#endif
