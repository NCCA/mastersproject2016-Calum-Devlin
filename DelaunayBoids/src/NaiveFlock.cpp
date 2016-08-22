#include <omp.h>

#include "NaiveFlock.h"

void NaiveFlock::debug()
{
  m_boids[0].debug(ngl::Vec3(10.1,0,5));
  m_boids[1].debug(ngl::Vec3(4.2,0,0));
  m_boids[2].debug(ngl::Vec3(10.3,0,-5.03));
  m_boids[3].debug(ngl::Vec3(-10.4,0,5.06));
  m_boids[4].debug(ngl::Vec3(-4.5,0,0));
  m_boids[5].debug(ngl::Vec3(-10.6,0,-5.09));

  m_boids[0].debug2(ngl::Vec3(-0.00251,0,0));
  m_boids[1].debug2(ngl::Vec3(-0.00252,0,0));
  m_boids[2].debug2(ngl::Vec3(-0.00253,0,0));
  m_boids[3].debug2(ngl::Vec3(0.00254,0,0));
  m_boids[4].debug2(ngl::Vec3(0.00255,0,0));
  m_boids[5].debug2(ngl::Vec3(0.00256,0,0));
}

NaiveFlock::NaiveFlock(const int *_flockSize, ngl::Vec3 _flockOrigin, const float *_velClamp, const float *_turnClamp, const float *_avoidRadius, const float *_approachRadius, const float *_fieldOfView, const int *_neighbourLimit)
  : Flock(_flockSize, _flockOrigin, _velClamp, _turnClamp, _avoidRadius, _approachRadius, _fieldOfView, _neighbourLimit)
{
  //debug();
}

NaiveFlock::~NaiveFlock()
{
  m_boids.clear();
}

void NaiveFlock::think()
{
  // Update the target
  moveTarget();

  #pragma omp parallel for
  for(int i = 0; i < m_boids.size(); ++i)
  {
    m_boids[i].clear();
    for(auto &neighbour : m_boids)
    {
      m_boids[i].think(neighbour);
    }
  }

  #pragma omp parallel for
  for(int i = 0; i < m_boids.size(); i++)
  {
    m_boids[i].move(m_target);
  }
}

void NaiveFlock::draw(const ngl::Mat4 &_globalTransformationMatrix)
{
  drawUniversalElements(_globalTransformationMatrix);
  drawSimpleBoids(_globalTransformationMatrix);
}
