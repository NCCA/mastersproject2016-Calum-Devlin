#ifndef DELAUNAYEDGE_H
#define DELAUNAYEDGE_H

#include "Boid.h"

class DelaunayEdge
{
  friend class DelFlock;
  public:
    DelaunayEdge(Boid *_boid1, Boid *_boid2);
    ~DelaunayEdge();

    void doubleBoidThink();
    int getID_1(){return m_boid_1->m_ID;}
    int getID_2(){return m_boid_2->m_ID;}

  private:
    Boid *m_boid_1;
    Boid *m_boid_2;
};

#endif // DELAUNAYEDGE_H
