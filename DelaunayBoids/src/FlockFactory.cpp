#include "FlockFactory.h"

#include "NaiveFlock.h"
#include "QuadFlock.h"
#include "DelFlock.h"

Flock *FlockFactory::GenerateFlock(FlockType _flocktype, const int *_flockSize, float _boidRadius, ngl::Vec3 _flockOrigin)
{
  if(_flocktype == FlockType::NAIVE)
  {
    return new NaiveFlock(_flockSize, _boidRadius, _flockOrigin);
  }

  if(_flocktype == FlockType::BINARY)
  {
    return new QuadFlock(_flockSize, _boidRadius, _flockOrigin);
  }

  if(_flocktype == FlockType::DELAUNAY)
  {
    //return new DelFlock(_flockSize, _boidRadius, _flockOrigin);
  }

  // Just a placeholder return statement until the other two are ready to go.
  return new NaiveFlock(_flockSize, _boidRadius, _flockOrigin);
}
