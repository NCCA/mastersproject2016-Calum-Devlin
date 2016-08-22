#ifndef BOID
#define BOID

#include <ngl/Vec3.h>
#include <ngl/Colour.h>
#include <ngl/Camera.h>
#include <vector>

class Flock;  class QuadTree;

class Boid
{
  friend class QuadTree;

  public:
    Boid();
    Boid(int _id, const float *_velClamp, const float *_turnClamp, const float *_avoidRadius, const float *_approachRadius, const float *_fieldOfView, const int *_neighbourLimit);
    ~Boid();

    void clear();
    void think(Boid &_neighbour);
    void move(ngl::Vec3 &_target);

    ngl::Mat4 getTransformation();
    ngl::Colour getColour();
    int m_ID;
    bool m_canMove;

    void debug(ngl::Vec3 pos);
    void debug2(ngl::Vec3 vel);
    void setSpecificPosVel(ngl::Vec3 _pos, ngl::Vec3 _vel);

    QuadTree *m_localRoot;

    ngl::Vec3 m_pos;

  private:
    const int *m_neighbourLimit;
    int m_currentNeighbourCount;

    ngl::Vec3 m_vel;
    ngl::Vec3 m_avoid;
    ngl::Vec3 m_approach;
    ngl::Vec3 m_align;

    ngl::Colour m_col;

    const float *m_velClamp;
    const float *m_turnClamp;
    const float *m_avoidRadius;
    const float *m_approachRadius;

    // m_fieldOfView is used for comparison against a dot product of normalised vectors.
    // Should be kept between [1,-1]
    const float *m_fieldOfView;
};

#endif // BOID
