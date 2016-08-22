#include <omp.h>

#include "QuadFlock.h"

void QuadFlock::debug()
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

QuadFlock::QuadFlock(const int *_flockSize, ngl::Vec3 _flockOrigin, const float *_velClamp, const float *_turnClamp, const float *_avoidRadius, const float *_approachRadius, const float *_fieldOfView, const int *_neighbourLimit)
  : Flock(_flockSize, _flockOrigin, _velClamp, _turnClamp, _avoidRadius, _approachRadius, _fieldOfView, _neighbourLimit)
{
  //debug();
  createQuadTree();
}

QuadFlock::~QuadFlock()
{
  m_boids.clear();
}

void QuadFlock::createQuadTree()
{
  m_flock.reset();
  QuadTree *thisFlock = new QuadTree(ngl::Vec3(0,0,0),2048.0f,2048.0f/*, getCam()*/);
  for(auto &boid : m_boids)
  {
    thisFlock->setDefaultLocalRoot(&boid);
  }
  m_flock.reset(thisFlock);
}

void QuadFlock::think()
{
  createQuadTree();
  moveTarget();

  #pragma omp parallel for
  //for(auto &boid : m_boids)
  for(int i = 0; i < m_boids.size(); ++i)
  {
    m_boids[i].clear();
  }

  #pragma omp parallel for
  //for(auto &boid : m_boids)
  for(int i = 0; i < m_boids.size(); ++i)
  {
    m_boids[i].m_localRoot->think(&m_boids[i]);
  }

  #pragma omp parallel for
  //for(auto &boid : m_boids)
  for(int i = 0; i < m_boids.size(); ++i)
  {
    m_boids[i].move(m_target);
  }
}

void QuadFlock::draw(const ngl::Mat4 &_globalTransformationMatrix)
{
  drawUniversalElements(_globalTransformationMatrix);
  drawSimpleBoids(_globalTransformationMatrix);

  ngl::Mat4 VVP = getCam()->getViewMatrix() * getCam()->getVPMatrix();
  m_flock->draw(_globalTransformationMatrix * VVP, m_inspectIndex);
}
