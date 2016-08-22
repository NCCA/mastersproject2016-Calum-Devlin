#ifndef DELAUNAYTRIANGLE_H
#define DELAUNAYTRIANGLE_H

#include "DelaunayEdge.h"

class DelaunayTriangle
{
  friend class DelFlock;

  public:
    DelaunayTriangle();
    DelaunayTriangle(Boid *_b1, Boid *_b2);
    DelaunayTriangle(Boid *_b1, Boid *_b2, Boid *_b3);

    void update(Boid *_b3);

  private:
    bool m_exists;
    //DelaunayEdge de1;
    //DelaunayEdge de2;
    //DelaunayEdge de3;

    Boid *b1;
    Boid *b2;
    Boid *b3;

    ngl::Vec3 v1;
    ngl::Vec3 v2;
    ngl::Vec3 v3;
    ngl::Vec3 m_circumcenter;

    // 2D vector cross method missing from ngl library.
    float cross2D(ngl::Vec2 u, ngl::Vec2 v);
};

#endif // DELAUNAYTRIANGLE_H
