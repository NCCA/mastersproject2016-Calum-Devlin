#ifndef QUADFLOCK_H
#define QUADFLOCK_H

#include "Flock.h"
#include "QuadTree.h"

class QuadFlock : public Flock
{
  public:
    QuadFlock(const int *_flockSize, float _boidRadius, ngl::Vec3 _flockOrigin);
    virtual ~QuadFlock();

    void debug();

    virtual void think();
    virtual void draw(const ngl::Mat4& _globalTransformationMatrix) const;

  private:
    const int *m_flockSize = Flock::m_flockSize;
    std::unique_ptr<QuadTree> m_flock;
};

#endif // QUADFLOCK_H
