#ifndef DELAUNAYTRIANGLE_H
#define DELAUNAYTRIANGLE_H

#include "DelaunayEdge.h"

class DelaunayTriangle
{
  friend class DelFlock;

  public:
    DelaunayTriangle();
    DelaunayTriangle(Boid *_b1, Boid *_b2, Boid *_b3);
    //DelaunayTriangle(DelaunayEdge _de1, DelaunayEdge _de2, DelaunayEdge _de3);

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
    float cross2D(ngl::Vec2 u, ngl::Vec2 v);
};

#endif // DELAUNAYTRIANGLE_H
