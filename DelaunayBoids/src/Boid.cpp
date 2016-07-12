#include <ngl/Transformation.h>
#include <ngl/VAOPrimitives.h>
#include <ngl/ShaderLib.h>
#include <ngl/Random.h>
#include <cmath>

#include "Flock.h"
#include "Boid.h"

ngl::Random *RNG = ngl::Random::instance();

Boid::Boid(int i, float _vClamp, float _tClamp, Flock *_flock)
{
  m_ID = i;
  m_pos = RNG->getRandomVec3();
  m_vel = _vClamp*RNG->getRandomVec3();
  m_pos.m_y = 0;
  m_vel.m_y = 0;
  m_col = RNG->getRandomColour();
  m_velClamp = _vClamp;
  m_turnClamp = _tClamp;
  m_flock = _flock;
}

Boid::~Boid()
{

}

void Boid::clear()
{
  //m_col = ngl::Colour(0,0,1);
  m_avoid = ngl::Vec3(0,0,0);
  m_approach = ngl::Vec3(0,0,0);
}

void Boid::update()
{
  m_pos += m_vel;
	ngl::Vec3 new_velocity = RNG->getRandomVec3();
  new_velocity.m_y=0;
  ngl::Vec3 j = new_velocity;
  j.normalize();
  ngl::Vec3 k = m_vel;
  k.normalize();
  float res = j.dot(k);
  if(res>m_turnClamp)
  {
    m_vel = new_velocity;
    m_vel.clamp(m_velClamp);
  }
}

void Boid::think(Boid &_neighbour)
{
	float distance = (m_pos - _neighbour.m_pos).length();

  // Sum the difference between all boids
  //if(distance < m_flock->m_boidRadius)
  if(distance < 9/*20*/ && distance > 3)
  {
    if(m_avoid == ngl::Vec3(0,0,0))
    {
      //m_col = ngl::Colour(0,1,0);
    }
    ngl::Vec3 approach = _neighbour.m_pos - m_pos;
    //approach.normalize();
    //approach *= 0.05;
    m_approach += approach;
    //std::cout<<approach.m_x<<" "<<approach.m_z<<"\n";
	}

  // Prioritise avoiding the nearest boid that is too close
  if(distance <= 3)
  {
    ngl::Vec3 avoid = m_pos - _neighbour.m_pos;
    //m_col = ngl::Colour(1,0,0);
    avoid /= avoid.length();
    if((m_avoid.length() > avoid.length()) || (m_avoid.length() == 0))
    {
      m_avoid = avoid;
    }
  }
}

void Boid::move()
{
  ngl::Vec3 m_targetTemp = (m_flock->m_target)-m_pos;
  m_targetTemp.normalize();

  if(m_avoid != ngl::Vec3(0,0,0))
  {
    //m_col = ngl::Colour(1,0,0);
    m_avoid.normalize();
    m_avoid *= 0.05*m_velClamp;
    m_vel += m_avoid;
  }
  else if(m_approach != ngl::Vec3(0,0,0))
  {
    //m_col = ngl::Colour(0,1,0);
    m_approach.normalize();
    m_approach *= 0.01*m_velClamp;
    m_vel += m_approach;
    m_vel += 0.01*m_velClamp*m_targetTemp;
  }
  else
  {
    //m_col = ngl::Colour(0,0,1);
    m_vel += 0.01*m_velClamp*m_targetTemp;
  }

  if(m_vel.length()>=0.1)
  {
    m_vel.normalize();
    m_vel *=0.1;
  }
  m_pos+=m_vel;
  //std::cout<<m_vel.m_x<<" "<<m_vel.m_z<<"\n";
  //m_pos+=m_intermediateVel;
}

void Boid::draw(const ngl::Mat4 &_globalTransformationMatrix) const
{
  ngl::Transformation transformation;
  transformation.setPosition(m_pos.m_x,m_pos.m_y,m_pos.m_z);
  // Retrieved from Stack Overflow to double check
  float pitchProjection = sqrt((m_vel.m_x*m_vel.m_x)+(m_vel.m_z*m_vel.m_z));
  float pitch = -atan2(m_vel.m_y,pitchProjection);
  float yaw = atan2(m_vel.m_x,m_vel.m_z);
  float roll = 0.0f;
  pitch = ngl::degrees(pitch);
  yaw = ngl::degrees(yaw);
  transformation.setRotation(pitch,yaw,roll);
  //transformation.setRotation(m_vel.m_x,m_vel.m_y,m_vel.m_z);

  ngl::Mat4 M = transformation.getMatrix() * _globalTransformationMatrix;
  ngl::Mat4 MV = M * m_flock->getCam()->getViewMatrix();
  ngl::Mat4 MVP = MV * m_flock->getCam()->getVPMatrix();
  ngl::Mat3 normalMatrix = MV;
  normalMatrix.inverse();

  ngl::ShaderLib *shader = ngl::ShaderLib::instance();
  shader->use("nglDiffuseShader");

  shader->setRegisteredUniform("M",M);
  shader->setRegisteredUniform("MV",MV);
  shader->setRegisteredUniform("MVP",MVP);
  shader->setRegisteredUniform("normalMatrix",normalMatrix);
  shader->setShaderParam4f("Colour",m_col.m_r,m_col.m_g,m_col.m_b,1);

  ngl::VAOPrimitives *prim = ngl::VAOPrimitives::instance();
  prim->draw("boid");
}
