#ifndef FLOCKFACTORY_H
#define FLOCKFACTORY_H

#include "Flock.h"

class FlockFactory
{
  public:
    Flock *GenerateFlock(FlockType _flocktype, const int *_flockSize, float _boidRadius, ngl::Vec3 _flockOrigin);
};

#endif // FLOCKFACTORY_H
