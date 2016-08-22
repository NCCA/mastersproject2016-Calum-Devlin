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
    DelFlock(const int *_flockSize, ngl::Vec3 _flockOrigin, const float* _velClamp, const float* _turnClamp, const float* _avoidRadius, const float* _approachRadius, const float* _fieldOfView, const int* _neighbourLimit);
    virtual ~DelFlock();

    virtual void think();
    virtual void draw(const ngl::Mat4& _globalTransformationMatrix);

  private:
    int m_delaunayEdges = 3* *m_flockSize - 7;
    std::vector<DelaunayEdge> m_edges;

    void setDelaunayIndices();
    bool isLeft(ngl::Vec3 *_from, ngl::Vec3 *_to, ngl::Vec3 *_side);
    bool DelaunayDiagonalTest(DelaunayTriangle *_PDTleft,DelaunayTriangle *_PDTright);

    std::unique_ptr<ngl::VertexArrayObject> edges;
};

#endif // DELFLOCK_H
