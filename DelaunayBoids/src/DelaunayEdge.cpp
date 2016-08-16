#include "include/DelaunayEdge.h"

DelaunayEdge::DelaunayEdge(Boid *_boid1, Boid *_boid2)
{
  m_boid_1 = _boid1;
  m_boid_2 = _boid2;
}

DelaunayEdge::~DelaunayEdge()
{

}

void DelaunayEdge::doubleBoidThink()
{
  m_boid_1->think(*m_boid_2);
  m_boid_2->think(*m_boid_1);
}
