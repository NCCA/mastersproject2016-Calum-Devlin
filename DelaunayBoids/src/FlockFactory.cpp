#include "FlockFactory.h"

#include "NaiveFlock.h"
#include "QuadFlock.h"
#include "DelFlock.h"

Flock *FlockFactory::GenerateFlock(FlockType _flocktype, const int *_flockSize, ngl::Vec3 _flockOrigin, const float* _velClamp, const float* _turnClamp, const float* _avoidRadius, const float* _approachRadius, const float* _fieldOfView, const int* _neighbourLimit)
{
  if(_flocktype == FlockType::NAIVE)
  {
    return new NaiveFlock(_flockSize, _flockOrigin, _velClamp, _turnClamp, _avoidRadius, _approachRadius, _fieldOfView, _neighbourLimit);
  }

  if(_flocktype == FlockType::BINARY)
  {
    return new QuadFlock(_flockSize, _flockOrigin, _velClamp, _turnClamp, _avoidRadius, _approachRadius, _fieldOfView, _neighbourLimit);
  }

  if(_flocktype == FlockType::DELAUNAY)
  {
    return new DelFlock(_flockSize, _flockOrigin, _velClamp, _turnClamp, _avoidRadius, _approachRadius, _fieldOfView, _neighbourLimit);
  }

  return new NaiveFlock(_flockSize, _flockOrigin, _velClamp, _turnClamp, _avoidRadius, _approachRadius, _fieldOfView, _neighbourLimit);
}
