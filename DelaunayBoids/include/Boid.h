#ifndef BOID
#define BOID

#include <ngl/Vec3.h>
#include <ngl/Colour.h>
#include <ngl/Camera.h>
#include <vector>

class Flock;  class QuadTree;

class Boid
{
  friend class QuadTree;

  public:
    Boid();
    Boid(Flock *_parent, int _id);
    ~Boid();

    void clear();
    void update();
    void think(Boid &_neighbour);
    void move(ngl::Vec3 &_target);
    //void draw(const ngl::Mat4 &_globalTransformationMatrix) const;

    ngl::Mat4 getTransformation();
    ngl::Colour getColour();
    int m_ID;
    bool m_canMove;

    void debug(ngl::Vec3 pos);
    void debug2(ngl::Vec3 vel);
    void setSpecificPosVel(ngl::Vec3 _pos, ngl::Vec3 _vel);

    QuadTree *m_localRoot;
    //std::vector<DelaunayEdge> m_localEdges;





    ngl::Vec3 m_pos;








  private:
    const Flock *m_flock;

    int m_neighbourLimit;
    int m_currentNeighbourCount;
    ngl::Vec3 m_vel;
    ngl::Vec3 m_avoid;
    ngl::Vec3 m_approach;
    ngl::Vec3 m_align;

    ngl::Colour m_col;

    float m_velClamp;
    float m_turnClamp;
    float m_avoidRadius;
    float m_approachRadius;

    // m_fieldOfView is used for comparison against a dot product of normalised vectors.
    // Should be kept between [1,-1]
    float m_fieldOfView;
};

#endif // BOID
