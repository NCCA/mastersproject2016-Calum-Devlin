#ifndef NAIVEFLOCK_H
#define NAIVEFLOCK_H

#include "Flock.h"

class NaiveFlock : public Flock
{
  public:
    NaiveFlock(const int *_flockSize, float _boidRadius, ngl::Vec3 _flockOrigin);
    virtual ~NaiveFlock();

    void debug();

    virtual void think();
    virtual void draw(const ngl::Mat4& _globalTransformationMatrix) const;

  private:
    const int *m_flockSize = Flock::m_flockSize;
};

#endif // NAIVEFLOCK_H
