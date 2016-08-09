#include <ngl/Transformation.h>
#include <ngl/VAOPrimitives.h>
#include <ngl/ShaderLib.h>
//#include <ngl/Random.h>

#include "DelFlock.h"

DelFlock::DelFlock(int _flockSize, float _boidRadius)
{
  //ngl::Random *RNG = ngl::Random::instance();
  m_flockSize = _flockSize;
  m_boidRadius = _boidRadius;
  for(int i = 0; i < m_flockSize; i++)
  {
    //                    vel  rot
    //Boid newBoid = Boid(i,0.05f,0.95,this);
    //m_boids.push_back(newBoid);
  }
  theta = 0;
}

/*void DelFlock::update()
{
  m_target = ngl::Vec3(200*sin(3*theta),20*sin(28*theta),200*cos(5*theta));
  for(int i = 0; i < m_flockSize; i++)
  {
    m_boids[i].clear();
    for(int j = 0; j < m_flockSize; j++)
    {
      // Avoid self-comparison
      if(i != j)
      {
        Boid &neighbour = m_boids[j];
        m_boids[i].think(neighbour);
      }
    }
  }
  for(int i = 0; i < m_flockSize; i++)
  {
    m_boids[i].move(m_target);
  }
  theta+=thetaStep;
  theta = fmod(theta,360);
}*/

void DelFlock::draw(const ngl::Mat4& _globalTransformationMatrix) const
{
  for(Boid b : m_boids)
  {
    //b.draw(_globalTransformationMatrix);
  }

  ngl::Transformation transformation;
  transformation.setPosition(m_target.m_x,m_target.m_y,m_target.m_z);

  ngl::Mat4 M = transformation.getMatrix() * _globalTransformationMatrix;
  ngl::Mat4 MV = M * getCam()->getViewMatrix();
  ngl::Mat4 MVP = MV * getCam()->getVPMatrix();
  ngl::Mat3 normalMatrix = MV;
  normalMatrix.inverse();

  ngl::ShaderLib *shader = ngl::ShaderLib::instance();
  shader->use("nglDiffuseShader");

  shader->setRegisteredUniform("M",M);
  shader->setRegisteredUniform("MV",MV);
  shader->setRegisteredUniform("MVP",MVP);
  shader->setRegisteredUniform("normalMatrix",normalMatrix);
  shader->setShaderParam4f("Colour",1,0,0,1);

  ngl::VAOPrimitives *prim = ngl::VAOPrimitives::instance();
  prim->draw("target");
}
