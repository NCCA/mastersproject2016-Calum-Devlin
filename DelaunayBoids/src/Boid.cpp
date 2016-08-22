#include <ngl/Transformation.h>
#include <ngl/VAOPrimitives.h>
#include <ngl/ShaderLib.h>
#include <ngl/Random.h>
#include <cmath>

#include "Boid.h"

ngl::Random *RNG = ngl::Random::instance();

// This is a specific method, used to pin 4 boids in the corners for the Delaunay flock.
void Boid::setSpecificPosVel(ngl::Vec3 _pos, ngl::Vec3 _vel)
{
  m_pos = _pos;
  m_vel = _vel;
  m_canMove = false;
}

void Boid::debug(ngl::Vec3 pos)
{
  m_pos = pos;
}

void Boid::debug2(ngl::Vec3 vel)
{
  m_vel = vel;
}

Boid::Boid()
{

}

Boid::Boid(int _id, const float *_velClamp, const float *_turnClamp, const float *_avoidRadius, const float *_approachRadius, const float *_fieldOfView, const int *_neighbourLimit)
{
  m_ID = _id;
  m_velClamp = _velClamp;
  m_turnClamp = _turnClamp;
  m_avoidRadius = _avoidRadius;
  m_approachRadius = _approachRadius;
  m_fieldOfView = _fieldOfView;
  m_neighbourLimit = _neighbourLimit;

  m_pos = 500*RNG->getRandomVec3();
  m_vel = 0.25*RNG->getRandomVec3();
  m_col = RNG->getRandomColour();
  m_pos.m_y = 0;
  m_vel.m_y = 0;
  m_canMove = true;
}

Boid::~Boid()
{

}

void Boid::clear()
{
  m_avoid = ngl::Vec3(0,0,0);
  m_approach = ngl::Vec3(0,0,0);
  m_align = m_vel;
  m_currentNeighbourCount = 0;
}

void Boid::think(Boid &_neighbour)
{
  ngl::Vec3 difference = _neighbour.m_pos - m_pos;

  // If the potential neighbour is in exactly the same position, it is likely to be the same boid
  // Therefore, we ignore it.
  if((difference.length() <= *m_approachRadius) && (difference.length() > 0))
  {
    // Field of View condition
    ngl::Vec3 j = m_vel;
    //ngl::Vec3 k = approach;
    ngl::Vec3 k = difference;
    j.normalize();
    k.normalize();
    ngl::Real dotProduct = j.dot(k);
    if(dotProduct > *m_fieldOfView)
    {
      ++m_currentNeighbourCount;
      // Sum the difference between all neighbour boids
      m_approach += difference;

      // Average the velocity of all neighbour boids for an alignment to aim for
      m_align += _neighbour.m_vel;

      if(difference.length() <= *m_avoidRadius)
      {
        if((m_avoid.length() > difference.length()) || (m_avoid.length() == 0))
        {
          m_avoid = -difference;
        }
      }
    }
	}
}

void Boid::move(ngl::Vec3 &_target)
{
  if(m_canMove)
  {
    ngl::Vec3 m_targetTemp = _target-m_pos;
    m_targetTemp.normalize();
    ngl::Vec3 m_newVel = m_vel;

    // Priorities:
    // Avoid nearest boid
    // Average position with nearest boids
    // Aim for the target / carry on as before
    if(m_avoid != ngl::Vec3(0,0,0))
    {
      //m_col = ngl::Colour(1,0,0);
      ngl::Vec3 alpha = m_avoid;
      ngl::Vec3 beta = m_newVel;

      alpha.normalize();
      beta.normalize();

      ngl::Vec3 reject = alpha - ((alpha.dot(beta)) * beta);
      reject.normalize();
      reject *= 0.04**m_velClamp;

      //m_newVel += reject;

      m_avoid.normalize();
      m_avoid *= 0.04**m_velClamp;

      m_newVel += m_avoid;

      m_align.normalize();
      m_align *= 0.02**m_velClamp;
      m_newVel += m_align;
    }
    else if((m_approach != ngl::Vec3(0,0,0)) && (m_currentNeighbourCount < *m_neighbourLimit))
    {
      //m_col = ngl::Colour(0,1,0);
      m_approach.normalize();
      m_approach *= 0.02**m_velClamp;
      m_newVel += m_approach;

      m_align.normalize();
      m_align *= 0.02**m_velClamp;
      m_newVel += m_align;
    }
    else
    {
      //m_col = ngl::Colour(0,0,1);
      m_newVel += 0.02**m_velClamp*m_targetTemp;
    }

    m_newVel += 0.02**m_velClamp*m_targetTemp;

    // Do not use the inbuilt clamp function.
    // It operates on all vector elements independently.

    // Limit maximum speed
    if(m_newVel.length() >= 2.5**m_velClamp)
    {
      m_newVel.normalize();
      m_newVel *= 2.5**m_velClamp;
    }
    // Limit minimum sped
    if(m_newVel.length() <= 0.01**m_velClamp)
    {
      if(m_newVel.length() != 0.0)
      {
        m_newVel.normalize();
        m_newVel *= 0.01**m_velClamp;
      }
    }

    m_pos+=m_newVel;
    m_vel = m_newVel;
  }
}

ngl::Mat4 Boid::getTransformation()
{
  ngl::Transformation transformation;

  // Set Translation
  transformation.setPosition(m_pos.m_x,m_pos.m_y,m_pos.m_z);

  // Convert velocity into pitch and yaw rotation.
  // Calculating roll would also require rate of change of velocity,
  // But we can get away with leaving roll as 0.
  float pitchProjection = sqrt((m_vel.m_x*m_vel.m_x)+(m_vel.m_z*m_vel.m_z));
  float pitch = -atan2(m_vel.m_y,pitchProjection);
  float yaw = atan2(m_vel.m_x,m_vel.m_z);
  float roll = 0.0f;
  pitch = ngl::degrees(pitch);
  yaw = ngl::degrees(yaw);
  roll = ngl::degrees(roll);
  transformation.setRotation(pitch,yaw,roll);

  return transformation.getMatrix();
}

ngl::Colour Boid::getColour()
{
  return m_col;
}
