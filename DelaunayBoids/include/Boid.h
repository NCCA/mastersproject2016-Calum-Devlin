#ifndef BOID
#define BOID

#include <ngl/Vec3.h>
#include <ngl/Colour.h>
#include <ngl/Camera.h>

class Flock;

class Boid
{
  public:
    Boid(int i, float _vClamp, float _tClamp, Flock *_flock);
    ~Boid();
    void clear();
    void update();
    void think(Boid &_neighbour);
    void move();
    void draw(const ngl::Mat4 &_globalTransformationMatrix) const;

  private:
    const Flock *m_flock;
    int m_ID;

    ngl::Vec3 m_pos;
    ngl::Vec3 m_vel;
    ngl::Vec3 m_avoid;
    ngl::Vec3 m_approach;

    ngl::Colour m_col;

    float m_velClamp;
    float m_turnClamp;
};

#endif // BOID
