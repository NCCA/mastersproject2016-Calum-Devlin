#include <ngl/Transformation.h>
#include <ngl/VAOPrimitives.h>
#include <ngl/ShaderLib.h>

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

NaiveFlock::NaiveFlock(const int *_flockSize, float _boidRadius, ngl::Vec3 _flockOrigin)
  : Flock(_flockSize, _boidRadius, _flockOrigin)
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
  m_target = ngl::Vec3(750*sin(10*theta),0*20*sin(28*theta),750*cos(10*theta));

  /*clock_t t;
  clock_t t2;
  t = clock();*/

  for(auto &boid : m_boids)
  {
    boid.clear();
    for(auto &neighbour : m_boids)
    {
      // Avoid self-comparison
      if(&boid != &neighbour)
      {
        boid.think(neighbour);
      }
    }
  }
  /*t2 = clock() - t;
  std::cout<<"T2:  "<<t2<<"\n";*/

  for(auto &boid : m_boids)
  {
    boid.move(m_target);
  }
}

void NaiveFlock::draw(const ngl::Mat4& _globalTransformationMatrix) const
{
  ngl::VAOPrimitives *prim = ngl::VAOPrimitives::instance();
  ngl::ShaderLib *shader = ngl::ShaderLib::instance();
  shader->use("nglDiffuseShader");

  ngl::Mat4 V = getCam()->getViewMatrix();
  ngl::Mat4 VP = getCam()->getVPMatrix();

  for(Boid b : m_boids)
  {
    ngl::Colour boidColour = b.getColour();
    ngl::Mat4 M = b.getTransformation() * _globalTransformationMatrix;
    ngl::Mat4 MV = M * V;
    ngl::Mat4 MVP = MV * VP;
    ngl::Mat3 normalMatrix = MV;
    normalMatrix.inverse();

    shader->setRegisteredUniform("M",M);
    shader->setRegisteredUniform("MV",MV);
    shader->setRegisteredUniform("MVP",MVP);
    shader->setRegisteredUniform("normalMatrix",normalMatrix);
    shader->setShaderParam4f("Colour",boidColour.m_r,boidColour.m_g,boidColour.m_b,1);

    prim->draw("boid");
  }

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
