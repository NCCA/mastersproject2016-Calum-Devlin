#ifndef QUADFLOCK_H
#define QUADFLOCK_H

#include "Flock.h"
#include "QuadTree.h"

class QuadFlock : public Flock
{
  public:
    QuadFlock(const int *_flockSize,
              ngl::Vec3 _flockOrigin,
              const float *_velClamp,
              const float *_turnClamp,
              const float *_avoidRadius,
              const float *_approachRadius,
              const float *_fieldOfView,
              const int *_neighbourLimit
              );
    virtual ~QuadFlock();

    void debug();

    void createQuadTree();
    virtual void think();
    virtual void draw(const ngl::Mat4& _globalTransformationMatrix);

  private:
    std::unique_ptr<QuadTree> m_flock;
};

#endif // QUADFLOCK_H
