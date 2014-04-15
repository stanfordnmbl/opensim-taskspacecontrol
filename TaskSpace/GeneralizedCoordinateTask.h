#ifndef OPENSIM_TASKSPACE_GENERALIZEDCOORDINATETASK_H_
#define OPENSIM_TASKSPACE_GENERALIZEDCOORDINATETASK_H_

using SimTK::State;
using SimTK::Matrix;
using SimTK::Vector;

namespace OpenSim {

namespace TaskSpace {

class OSIMTASKSPACE_API GeneralizedCoordinateTask : public TaskSpace::Task
{
OpenSim_DECLARE_ABSTRACT_OBJECT(GeneralizedCoordinateTask, TaskSpace::Task);

public:
    /** @name Property declarations */
    /**@{**/
    // TODO ignoreCoriolis
    // TODO ignoreGravity
    /**@}**/

    /// Identity.
    Matrix nullspaceProjectionTranspose(const State& s) const
    {
        // TODO could be created at compile time.
        Matrix NT(_numCoords);
        NT.setToZeros();
        NT.diag().setTo(1.0);
        /*
        for (unsigned int i = 0; i < _numCoords; i++)
        {
            NT[i, i] = 1.0;
        }
        */
        return NT;
    }

    virtual Vector generalizedForces(const State& s) const = 0;

};

} // namespace TaskSpace

} // namespace OpenSim

#endif
