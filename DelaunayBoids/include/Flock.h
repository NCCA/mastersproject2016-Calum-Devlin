#ifndef FLOCK_H
#define FLOCK_H

#include <vector>
#include <ngl/Camera.h>

#include <Boid.h>

class Flock
{
  public:
    Flock(int _flockSize, float _boidRadius);
    int m_flockSize;
    float m_boidRadius;
    ngl::Vec3 m_target;

    void update();
    void draw(const ngl::Mat4 &_globalTransformationMatrix) const;

    inline void setCam(ngl::Camera *_cam){m_cam=_cam;}
    inline ngl::Camera *getCam()const {return m_cam;}
    inline void setShaderName(const std::string &_n){m_shaderName=_n;}
    inline const std::string getShaderName()const {return m_shaderName;}
  private:
    std::string m_shaderName;
    // Camera pointer
    ngl::Camera *m_cam;

    std::vector<Boid> m_boids;

    float theta;
    const float thetaStep = 0.0001;
};

#endif // FLOCK_H
