#ifndef FLOCK_H
#define FLOCK_H

#include <vector>
#include <Boid.h>
//#include <memory>
#include <iostream>

enum class FlockType
{
  NAIVE, BINARY, DELAUNAY
};

class Flock
{
  friend class NaiveFlock;
  friend class QuadFlock;
  friend class DelFlock;

  public:
    Flock(const int *_flockSize, float _boidRadius, ngl::Vec3 _flockOrigin);
    virtual ~Flock();

    virtual void think() = 0;
    virtual void draw(const ngl::Mat4& _globalTransformationMatrix) const = 0;

    inline void setCam(ngl::Camera *_cam){m_cam=_cam;}
    inline ngl::Camera *getCam()const {return m_cam;}
    inline void setShaderName(const std::string &_n){m_shaderName=_n;}
    inline const std::string getShaderName()const {return m_shaderName;}

  protected:
    std::vector<Boid> m_boids;
    //std::vector<DELEDGES> m_edges;

    // Used to make a moving, periodic target for the boids to follow
    float theta = 0.0f;
    const/*expr*/ float thetaStep = 0.002f;
    ngl::Vec3 m_target;

    inline int getFlockSize() {return *m_flockSize;}
    void moveTarget();

    float m_velClamp;
    float m_turnClamp;
    float m_avoidRadius;
    float m_approachRadius;

  private:
    const int *m_flockSize;
    float m_boidRadius = 1.0f;
    ngl::Vec3 m_flockOrigin = ngl::Vec3(0,0,0);

    // Camera Pointer
    ngl::Camera *m_cam;
    // Shader Reference
    std::string m_shaderName;
};

#endif // FLOCK_H
