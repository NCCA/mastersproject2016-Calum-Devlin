#ifndef FLOCK_H
#define FLOCK_H

#include <vector>
#include "Boid.h"

enum class FlockType
{
  NAIVE, BINARY, DELAUNAY
};

class Flock
{
  friend class NaiveFlock;
  friend class QuadFlock;
  friend class DelFlock;
  friend class Boid;

  public:
    Flock(const int *_flockSize,
          ngl::Vec3 _flockOrigin,
          const float *_velClamp,
          const float *_turnClamp,
          const float *_avoidRadius,
          const float *_approachRadius,
          const float *_fieldOfView,
          const int *_neighbourLimit
          );
    virtual ~Flock();

    virtual void think() = 0;
    virtual void draw(const ngl::Mat4 &_globalTransformationMatrix) = 0;
    void drawUniversalElements(const ngl::Mat4& _globalTransformationMatrix);
    void drawSimpleBoids(const ngl::Mat4& _globalTransformationMatrix);

    inline void setCam(ngl::Camera *_cam){m_cam=_cam;}
    inline ngl::Camera *getCam()const {return m_cam;}
    inline void setShaderName(const std::string &_n){m_shaderName=_n;}
    inline const std::string getShaderName()const {return m_shaderName;}

    void inspectBoid(int _ID);

  protected:
    std::vector<Boid> m_boids;
    int m_inspectIndex = -1;

    // Used to make a moving, periodic target for the boids to follow
    float theta = 0.0f;
    const float thetaStep = 0.002f;
    ngl::Vec3 m_target;

    inline int getFlockSize() {return *m_flockSize;}
    void moveTarget();

  private:
    ngl::Vec3 m_flockOrigin = ngl::Vec3(0,0,0);
    const int *m_flockSize;
    const float *m_velClamp;
    const float *m_turnClamp;
    const float *m_avoidRadius;
    const float *m_approachRadius;
    const float *m_fieldOfView;
    const int *m_neighbourLimit;

    // Camera Pointer
    ngl::Camera *m_cam;
    // Shader Reference
    std::string m_shaderName;
};

#endif // FLOCK_H
