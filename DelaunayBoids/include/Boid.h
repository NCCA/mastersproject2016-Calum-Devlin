#ifndef BOID
#define BOID

#include <ngl/Vec3.h>
#include <ngl/Colour.h>
#include <ngl/Camera.h>

class Flock;  class QuadTree;

class Boid
{
  friend class QuadTree;

  public:
    Boid(Flock *_parent, int _id);
    ~Boid();

    void clear();
    void update();
    void think(Boid &_neighbour);
    void move(ngl::Vec3 &_target);
    //void draw(const ngl::Mat4 &_globalTransformationMatrix) const;

    ngl::Mat4 getTransformation();
    ngl::Colour getColour();

    void debug(ngl::Vec3 pos);
    void debug2(ngl::Vec3 pos);

    QuadTree *m_localRoot;

  private:
    const Flock *m_flock;
    int m_ID;

    ngl::Vec3 m_pos;
    ngl::Vec3 m_vel;
    ngl::Vec3 m_avoid;
    ngl::Vec3 m_approach;
    ngl::Vec3 m_align;

    ngl::Colour m_col;

    float m_velClamp;
    float m_turnClamp;
    float m_avoidRadius;
    float m_approachRadius;

    // m_fieldOfView is used for comparison against a dot product of normalised vectors.
    // Should be kept between [1,-1]
    float m_fieldOfView;
};

#endif // BOID
