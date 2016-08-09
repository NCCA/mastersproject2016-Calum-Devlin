#include "Flock.h"

Flock::Flock(const int *_flockSize, float _boidRadius, ngl::Vec3 _flockOrigin)
{
  m_flockSize = _flockSize;
  m_boidRadius = _boidRadius;
  m_flockOrigin = _flockOrigin;
  for(int i = 0; i < *m_flockSize; i++)
  {
    Boid newBoid = Boid(this,i);
    m_boids.push_back(newBoid);
  }
}

Flock::~Flock()
{
  std::cout<<"Culling the flock.\n";
}

void Flock::moveTarget()
{
  theta+=thetaStep;
  theta = fmod(theta,360);
}
