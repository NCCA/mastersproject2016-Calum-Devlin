#include "include/DelaunayEdge.h"
#include <omp.h>

DelaunayEdge::DelaunayEdge(Boid *_boid1, Boid *_boid2)
{
  m_boid_1 = _boid1;
  m_boid_2 = _boid2;
}

DelaunayEdge::DelaunayEdge(Boid *_boid1, Boid *_boid2, Boid *_boidLeft, Boid *_boidRight)
{
  m_boid_1 = _boid1;
  m_boid_2 = _boid2;
  m_boid_left = _boidLeft;
  m_boid_right = _boidRight;
}

DelaunayEdge::~DelaunayEdge()
{

}

void DelaunayEdge::doubleBoidThink()
{
  if(m_boid_1->m_canMove && m_boid_2->m_canMove)
  {
    #pragma omp parallel
    {
      m_boid_1->think(*m_boid_2);
      m_boid_2->think(*m_boid_1);
    }
  }
}
