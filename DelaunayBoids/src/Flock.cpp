#include <ngl/Transformation.h>
#include <ngl/VAOPrimitives.h>
#include <ngl/ShaderLib.h>

#include "Flock.h"

Flock::Flock(const int *_flockSize, ngl::Vec3 _flockOrigin, const float* _velClamp, const float* _turnClamp, const float* _avoidRadius, const float* _approachRadius, const float* _fieldOfView, const int* _neighbourLimit)
{
  m_flockSize = _flockSize;
  m_flockOrigin = _flockOrigin;
  m_velClamp = _velClamp;
  m_turnClamp = _turnClamp;
  m_avoidRadius = _avoidRadius;
  m_approachRadius = _approachRadius;
  m_fieldOfView = _fieldOfView;
  m_neighbourLimit = _neighbourLimit;

  for(int i = 0; i < *m_flockSize; i++)
  {
    Boid newBoid = Boid(i, m_velClamp, m_turnClamp, m_avoidRadius, m_approachRadius, m_fieldOfView, m_neighbourLimit);
    m_boids.push_back(newBoid);
  }
}

Flock::~Flock()
{
  std::cout<<"Culling the flock.\n";
}

void Flock::drawUniversalElements(const ngl::Mat4 &_globalTransformationMatrix)
{
  ngl::VAOPrimitives *prim = ngl::VAOPrimitives::instance();
  ngl::ShaderLib *shader = ngl::ShaderLib::instance();
  shader->use("nglDiffuseShader");

  ngl::Transformation target;
  target.setPosition(m_target);
  ngl::Mat4 V = getCam()->getViewMatrix();
  ngl::Mat4 VVP = V * getCam()->getVPMatrix();
  ngl::Mat4 MVP = target.getMatrix() * _globalTransformationMatrix * VVP;

  shader->setRegisteredUniform("MVP",MVP);
  shader->setShaderParam4f("Colour",1,0,0,1);

  prim->draw("target");
}

void Flock::drawSimpleBoids(const ngl::Mat4 &_globalTransformationMatrix)
{
  ngl::VAOPrimitives *prim = ngl::VAOPrimitives::instance();
  ngl::ShaderLib *shader = ngl::ShaderLib::instance();
  shader->use("nglDiffuseShader");

  ngl::Mat4 VVP = getCam()->getViewMatrix() * getCam()->getVPMatrix();

  for(Boid b : m_boids)
  {
    if(b.m_canMove)
    {
      ngl::Colour boidColour = b.getColour();
      ngl::Mat4 M = b.getTransformation();
      if(b.m_ID == m_inspectIndex)
      {
        boidColour = ngl::Colour(1,1,1,1);
        ngl::Transformation newScale;
        newScale.setScale(10,10,10);
        M *= newScale.getMatrix();
      }

      ngl::Mat4 MVP = M * _globalTransformationMatrix * VVP;

      shader->setRegisteredUniform("MVP",MVP);
      shader->setShaderParam4f("Colour",boidColour.m_r,boidColour.m_g,boidColour.m_b,1);

      prim->draw("boid");
    }
  }
}

void Flock::moveTarget()
{
  theta+=thetaStep;
  theta = fmod(theta,360);
  m_target = ngl::Vec3(750*sin(theta),0,750*cos(theta));
}

void Flock::inspectBoid(int _ID)
{
  m_inspectIndex = _ID;
}
