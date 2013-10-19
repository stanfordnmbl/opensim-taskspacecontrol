#include "PositionTask.h"

// TODO make these in-line for speed-up.
Vec3 TaskSpace::PositionTask::positionInGround(const State& s) const
{
    return _mobilizedBody.findStationLocationInGround(s, _positionInBody);
}

Vec3 TaskSpace::PositionTask::velocityInGround(const State& s) const
{
    return smss.multiplyByStationJacobian(s, _mobilizedBodyIndex,
            _positionInBody, s.getU());
}

Vector TaskSpace::PositionTask::controlLaw(const State& s) const
{ 
    return desiredAcceleration(s)
        + _velocityGain * (desiredVelocity(s) - velocityInGround(s))
        + _positionGain * (desiredPosition(s) - positionInGround(s));
}
