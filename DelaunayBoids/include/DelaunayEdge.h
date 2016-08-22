#ifndef DELAUNAYEDGE_H
#define DELAUNAYEDGE_H

#include "Boid.h"

class DelaunayEdge
{
  friend class DelFlock;
  public:
    DelaunayEdge(Boid *_boid1, Boid *_boid2);
    DelaunayEdge(Boid *_boid1, Boid *_boid2, Boid *_boidLeft, Boid *_boidRight);
    ~DelaunayEdge();

    void doubleBoidThink();
    int getID_1(){return m_boid_1->m_ID;}
    int getID_2(){return m_boid_2->m_ID;}

  private:
    Boid *m_boid_1;
    Boid *m_boid_2;
    Boid *m_boid_left;
    Boid *m_boid_right;

};

#endif // DELAUNAYEDGE_H
