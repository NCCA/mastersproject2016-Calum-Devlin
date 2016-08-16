#ifndef DELFLOCK_H
#define DELFLOCK_H

#include <vector>
#include <ngl/VertexArrayObject.h>

#include "Flock.h"
#include "DelaunayEdge.h"
#include "DelaunayTriangle.h"

class DelFlock : public Flock
{
  public:
    DelFlock(const int *_flockSize, float _boidRadius, ngl::Vec3 _flockOrigin);
    virtual ~DelFlock();

    //void debug();

    virtual void think();
    virtual void draw(const ngl::Mat4 &_globalTransformationMatrix) const;

  private:
    const int *m_flockSize = Flock::m_flockSize;
    int m_delaunayEdges = 3* *m_flockSize - 7;
    std::vector<DelaunayEdge> m_edges;
    std::vector<Boid> m_cornerBoids;

    void setDelaunayIndices();
    bool isLeft(ngl::Vec3 *_from, ngl::Vec3 *_to, ngl::Vec3 *_side);

    std::unique_ptr<ngl::VertexArrayObject> edges;
};

#endif // DELFLOCK_H
