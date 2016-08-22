#ifndef NAIVEFLOCK_H
#define NAIVEFLOCK_H

#include "Flock.h"

class NaiveFlock : public Flock
{
  public:
    NaiveFlock(const int *_flockSize,
               ngl::Vec3 _flockOrigin,
               const float *_velClamp,
               const float *_turnClamp,
               const float *_avoidRadius,
               const float *_approachRadius,
               const float *_fieldOfView,
               const int *_neighbourLimit
               );
    virtual ~NaiveFlock();

    void debug();

    virtual void think();
    virtual void draw(const ngl::Mat4& _globalTransformationMatrix);
};

#endif // NAIVEFLOCK_H
