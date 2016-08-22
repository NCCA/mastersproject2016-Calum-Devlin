#include "DelaunayTriangle.h"

DelaunayTriangle::DelaunayTriangle()
{
  m_exists = false;
}

DelaunayTriangle::DelaunayTriangle(Boid *_b1, Boid *_b2)
{
  m_exists = false;
  b1 = _b1;
  b2 = _b2;
}

DelaunayTriangle::DelaunayTriangle(Boid *_b1, Boid *_b2, Boid *_b3)
{
  m_exists = true;
  // The three boids represent the three corners of the potential delaunay triangle
  b1 = _b1;
  b2 = _b2;
  b3 = _b3;

  // Mid-points between each pair of boids
  //ngl::Vec3 mp3 = (b1->m_pos + b2->m_pos)/2;
  ngl::Vec3 mp2 = (b1->m_pos + b3->m_pos)/2;
  ngl::Vec3 mp1 = (b2->m_pos + b3->m_pos)/2;

  // Vectors from the mid points towards the corner boids
  ngl::Vec3 mp12 = b2->m_pos - mp1;
  ngl::Vec3 mp23 = b3->m_pos - mp2;
  //ngl::Vec3 mp31 = b1->m_pos - mp3;

  //ngl::Vec3 mp13 = b3->m_pos - mp1;
  //ngl::Vec3 mp21 = b1->m_pos - mp2;
  //ngl::Vec3 mp32 = b2->m_pos - mp3;

  // Vectors that perpendicularly bisect each edge.
  // Beacuase we have no knowledge of what order the boids will be presented,
  // there are vectors towards and away from the circumcenter
  ngl::Vec3 bisector1A = ngl::Vec3(mp12.m_z,0,-mp12.m_x);
  ngl::Vec3 bisector2A = ngl::Vec3(mp23.m_z,0,-mp23.m_x);
  //ngl::Vec3 bisector3A = ngl::Vec3(mp31.m_z,0,-mp31.m_x);

  //ngl::Vec3 bisector1B = ngl::Vec3(mp13.m_z,0,-mp13.m_x);
  //ngl::Vec3 bisector2B = ngl::Vec3(mp21.m_z,0,-mp21.m_x);
  //ngl::Vec3 bisector3B = ngl::Vec3(mp32.m_z,0,-mp32.m_x);

  // The mathematics is converted from the StackOverflow thread here:
  // http://www.stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
  // Some error cases have been ignored, due to the assumption that this is a valid triangle
  // All vectors of the form *2 give the same answer, the circumcenter
  //ngl::Real u = ((mp2-mp1).cross(bisector1A)/(bisector1A.cross(bisector2A))).m_y;
  ngl::Real v = ((mp2-mp1).cross(bisector2A)/(bisector1A.cross(bisector2A))).m_y;
  ngl::Vec3 u2 = mp1 + v * bisector1A;
  /*ngl::Vec3 v2 = mp2 + u * bisector2A;

  ngl::Real w = ((mp3-mp1).cross(bisector1A)/(bisector1A.cross(bisector3A))).m_y;
  ngl::Real x = ((mp3-mp1).cross(bisector3A)/(bisector1A.cross(bisector3A))).m_y;

  ngl::Vec3 w2 = mp1 + x * bisector1A;
  ngl::Vec3 x2 = mp3 + w * bisector3A;

  ngl::Real y = ((mp2-mp3).cross(bisector3A)/(bisector3A.cross(bisector2A))).m_y;
  ngl::Real z = ((mp2-mp3).cross(bisector2A)/(bisector3A.cross(bisector2A))).m_y;

  ngl::Vec3 y2 = mp3 + z * bisector3A;
  ngl::Vec3 z2 = mp2 + y * bisector2A;*/

  m_circumcenter = u2;
}

// An abbreviation of the above method when 1 boid changes
void DelaunayTriangle::update(Boid *_b3)
{
  m_exists = true;
  b3 = _b3;

  ngl::Vec3 mp2 = (b1->m_pos + b3->m_pos)/2;
  ngl::Vec3 mp1 = (b2->m_pos + b3->m_pos)/2;

  ngl::Vec3 mp12 = b2->m_pos - mp1;
  ngl::Vec3 mp23 = b3->m_pos - mp2;

  ngl::Vec3 bisector1A = ngl::Vec3(mp12.m_z,0,-mp12.m_x);
  ngl::Vec3 bisector2A = ngl::Vec3(mp23.m_z,0,-mp23.m_x);

  ngl::Real v = ((mp2-mp1).cross(bisector2A)/(bisector1A.cross(bisector2A))).m_y;
  m_circumcenter = mp1 + v * bisector1A;
}

float DelaunayTriangle::cross2D(ngl::Vec2 u, ngl::Vec2 v)
{
  return(u.m_x*v.m_y) - (v.m_x*u.m_y);
}
