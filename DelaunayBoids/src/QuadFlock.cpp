#include <ngl/Transformation.h>
#include <ngl/VAOPrimitives.h>
#include <ngl/ShaderLib.h>

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

QuadFlock::QuadFlock(const int *_flockSize, float _boidRadius, ngl::Vec3 _flockOrigin)
  : Flock(_flockSize, _boidRadius, _flockOrigin)
{
  //debug();
}

QuadFlock::~QuadFlock()
{
  m_boids.clear();
}

void QuadFlock::think()
{
  QuadTree *thisFlock = new QuadTree(ngl::Vec3(0,0,0),2048.0f,2048.0f/*, getCam()*/);
  for(auto &boid : m_boids)
  {
    thisFlock->setDefaultLocalRoot(&boid);
  }
  m_flock.reset(thisFlock);

  moveTarget();
  m_target = ngl::Vec3(750*sin(theta),0*20*sin(3*theta),750*cos(theta));

  //m_flock->think();

  for(auto &boid : m_boids)
  {
    boid.clear();
  }

  for(auto &boid : m_boids)
  {
    boid.m_localRoot->think(&boid);
    //m_flock->think(&boid);
  }

  for(auto &boid : m_boids)
  {
    boid.move(m_target);
  }
}

void QuadFlock::draw(const ngl::Mat4 &_globalTransformationMatrix) const
{
  ngl::VAOPrimitives *prim = ngl::VAOPrimitives::instance();
  ngl::ShaderLib *shader = ngl::ShaderLib::instance();
  shader->use("nglDiffuseShader");

  ngl::Mat4 V = getCam()->getViewMatrix();
  ngl::Mat4 VP = getCam()->getVPMatrix();

  m_flock->draw(_globalTransformationMatrix, V, VP);

  // Also draw the target that the boids are following
  ngl::Transformation transformation;
  transformation.setPosition(m_target.m_x,m_target.m_y,m_target.m_z);

  ngl::Mat4 M = transformation.getMatrix() * _globalTransformationMatrix;
  ngl::Mat4 MV = M * V;
  ngl::Mat4 MVP = MV * VP;
  ngl::Mat3 normalMatrix = MV;
  normalMatrix.inverse();

  shader->setRegisteredUniform("M",M);
  shader->setRegisteredUniform("MV",MV);
  shader->setRegisteredUniform("MVP",MVP);
  shader->setRegisteredUniform("normalMatrix",normalMatrix);
  shader->setShaderParam4f("Colour",1,0,0,1);

  prim->draw("target");
}
