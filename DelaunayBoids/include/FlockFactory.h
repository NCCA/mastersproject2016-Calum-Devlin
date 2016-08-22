#ifndef FLOCKFACTORY_H
#define FLOCKFACTORY_H

#include "Flock.h"

class FlockFactory
{
  public:
    Flock *GenerateFlock(FlockType _flocktype,
                         const int *_flockSize,
                         ngl::Vec3 _flockOrigin,
                         const float *_velClamp,
                         const float *_turnClamp,
                         const float *_avoidRadius,
                         const float *_approachRadius,
                         const float *_fieldOfView,
                         const int *_neighbourLimit
                         );
};

#endif // FLOCKFACTORY_H
