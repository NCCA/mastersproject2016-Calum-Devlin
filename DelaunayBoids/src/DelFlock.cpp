#include <ngl/Transformation.h>
#include <ngl/VAOPrimitives.h>
#include <ngl/ShaderLib.h>
#include <omp.h>

#include "DelFlock.h"

DelFlock::DelFlock(const int *_flockSize, ngl::Vec3 _flockOrigin, const float *_velClamp, const float *_turnClamp, const float *_avoidRadius, const float *_approachRadius, const float *_fieldOfView, const int *_neighbourLimit)
  : Flock(_flockSize, _flockOrigin, _velClamp, _turnClamp, _avoidRadius, _approachRadius, _fieldOfView, _neighbourLimit)
{
  edges.reset(ngl::VertexArrayObject::createVOA(GL_LINES));

  //debug();
  // Construct the Delaunay Graph

  // Borrow the first four boids to pin the boundary of the delaunay graph
  // Not strictly speaking necessary, but it gives us some constants to work with
  m_boids[0].setSpecificPosVel(ngl::Vec3(1025,0,1025),ngl::Vec3(0,0,0));
  m_boids[1].setSpecificPosVel(ngl::Vec3(1026,0,-1026),ngl::Vec3(0,0,0));
  m_boids[2].setSpecificPosVel(ngl::Vec3(-1027,0,1027),ngl::Vec3(0,0,0));
  m_boids[3].setSpecificPosVel(ngl::Vec3(-1028,0,-1028),ngl::Vec3(0,0,0));

  for(auto &boidA : m_boids)
  {
    for(auto &boidB : m_boids)
    {
      if(&boidA.m_ID > &boidB.m_ID)
      {
        int leftBoids = 0;
        int rightBoids = 0;

        DelaunayTriangle PDTleft = DelaunayTriangle(&boidA, &boidB);// Potential Delaunay Traingle
        DelaunayTriangle PDTright = DelaunayTriangle(&boidA, &boidB);// Potential Delaunay Traingle

        for(auto &boidC : m_boids)
        {
          if((&boidA.m_ID != &boidC.m_ID)
             && (&boidB.m_ID != &boidC.m_ID))
          {
            // For future work: be careful with this method, as I think there's a co-linear loss of precision that induces misbehaviour
            bool leftVSright = isLeft(&boidA.m_pos, &boidB.m_pos, &boidC.m_pos);
            if(leftVSright)
            {
              // If it's the first boid encountered on this side, make a new delaunay triangle
              if(leftBoids == 0)
              {
                PDTleft.update(&boidC);
                ++leftBoids;
              }
              else
              {
                // When there is an existing delaunay triangle,
                // if the new boid is inside the circumRadius, replace with a smaller delaunay triangle
                float circumRadius = (PDTleft.m_circumcenter - boidA.m_pos).length();
                float circumCenterDistance = (PDTleft.m_circumcenter - boidC.m_pos).length();
                if(circumCenterDistance < circumRadius)
                {
                  PDTleft.update(&boidC);
                }
              }
              /*// In either case, check if this boid invalidates the delaunay triangle on the other side
              float circumRadius = (PDTright.m_circumcenter - boidA.m_pos).length();
              float circumCenterDistance = (PDTright.m_circumcenter - boidC.m_pos).length();
              if(circumCenterDistance < circumRadius)
              {
                //PDTright = DelaunayTriangle();
                //rightBoids = 0;
              }*/
            }
            else
            {
              // If it's the first boid encountered on this side, make a new delaunay triangle
              if(rightBoids == 0)
              {
                PDTright.update(&boidC);
                ++rightBoids;
              }
              else
              {
                // When there is an existing delaunay triangle,
                // if the new boid is inside the circumRadius, replace with a smaller delaunay triangle
                float circumRadius = (PDTright.m_circumcenter - boidA.m_pos).length();
                float circumCenterDistance = (PDTright.m_circumcenter - boidC.m_pos).length();
                if(circumCenterDistance < circumRadius)
                {
                  PDTright.update(&boidC);
                }
              }
              /*// In either case, check if this boid invalidates the delaunay triangle on the other side
              float circumRadius = (PDTleft.m_circumcenter - boidA.m_pos).length();
              float circumCenterDistance = (PDTleft.m_circumcenter - boidC.m_pos).length();
              if(circumCenterDistance < circumRadius)
              {
                //PDTleft = DelaunayTriangle();
                //leftBoids = 0;
              }*/
            }
            // NEVER remove this without drastically reducing the flocksize, or you'll be here all day
            bool validDelaunayEdge = DelaunayDiagonalTest(&PDTleft, &PDTright);
            if(!validDelaunayEdge)
            {
              break;
            }
          }
        }

        if(PDTleft.m_exists && PDTright.m_exists)
        {
          bool validDelaunayEdge = DelaunayDiagonalTest(&PDTleft, &PDTright);
          if(validDelaunayEdge)
          {
            DelaunayEdge de1 = DelaunayEdge(PDTleft.b1, PDTleft.b2, PDTleft.b3, PDTright.b3);
            m_edges.push_back(de1);
          }
        }
        else
        {
          // Four boundary cases
          if(PDTleft.m_exists)
          {
            DelaunayEdge de1 = DelaunayEdge(PDTleft.b1, PDTleft.b2);
            m_edges.push_back(de1);
          }
          if(PDTright.m_exists)
          {
            DelaunayEdge de1 = DelaunayEdge(PDTright.b1, PDTright.b2);
            m_edges.push_back(de1);
          }
        }
      }
    }
  }
}

DelFlock::~DelFlock()
{
  m_boids.clear();
  m_edges.clear();
}

void DelFlock::think()
{
  moveTarget();

  #pragma omp parallel for
  for(int i = 0; i < m_boids.size(); ++i)
  {
    m_boids[i].clear();
  }

  for(auto &edge : m_edges)
  {
    edge.doubleBoidThink();
  }

  #pragma omp parallel for
  for(int i = 0; i < m_boids.size(); i++)
  {
    m_boids[i].move(m_target);
  }

  /*for(auto &edge : m_edges)
  {
    // Test if the edge can flip.  This could be fast if we are starting from a near-Delaunay triangulation
  }*/
}

/****************************************************************/
bool DelFlock::isLeft(ngl::Vec3 *_from, ngl::Vec3 *_to, ngl::Vec3 *_side)
{
  ngl::Real a = (_to->m_x - _from->m_x)*(_side->m_z - _from->m_z);
  ngl::Real b = (_to->m_z - _from->m_z)*(_side->m_x - _from->m_x);
  if(a==b)
  {
    std::cout<<"BEWARE THE LINEAR CASE.\n";
    std::cout<<" Three of your boids are in an almost straight line, and due to rounding errors, cannot be distinguished. ";
    std::cout<<" But you may get lucky, and have the correct edges calculated anyway.\n";
  }
  return (a > b);
}

// For a quadrilateral composed of two PotentialDelaunayTriangles which share a diagonal
// returns True if the diagonal remains a valid DelaunayEdge,
// or False if the other diagonal is more suited.
// Also catches the boundary case where at least one PDT has not been constructed yet.
bool DelFlock::DelaunayDiagonalTest(DelaunayTriangle *_PDTleft, DelaunayTriangle *_PDTright)
{
  if(_PDTleft->m_exists && _PDTright->m_exists)
  {
    // Use any boid in the PDT to work out the CircumRadius of that PDT
    // Take the non-overlapping boid from the other PDT and calculate its distance from the circumcenter
    float circumRadiusLeft = (_PDTleft->m_circumcenter - _PDTleft->b1->m_pos).length();
    float circumCenterDistanceLeft = (_PDTleft->m_circumcenter - _PDTright->b3->m_pos).length();

    // Swap the PDT's and repeat
    float circumRadiusRight = (_PDTright->m_circumcenter - _PDTright->b1->m_pos).length();
    float circumCenterDistanceRight = (_PDTright->m_circumcenter - _PDTleft->b3->m_pos).length();

    if((circumCenterDistanceLeft > circumRadiusLeft)
       && (circumCenterDistanceRight > circumRadiusRight))
    {
      return true;
    }
    return false;
  }
  return true;
}

void DelFlock::draw(const ngl::Mat4 &_globalTransformationMatrix)
{
  drawUniversalElements(_globalTransformationMatrix);
  drawSimpleBoids(_globalTransformationMatrix);

  // DRAW THE EDGES SPECIFIC TO THIS TYPE OF FLOCK

  ngl::ShaderLib *shader = ngl::ShaderLib::instance();
  shader->use("nglDiffuseShader");

  ngl::Mat4 MVP = _globalTransformationMatrix * getCam()->getViewMatrix() * getCam()->getVPMatrix();
  shader->setRegisteredUniform("MVP",MVP);

  shader->setShaderParam4f("Colour",0,0,0,1);

  // A Magic number, assuming four boids define a quadrilateral convex hull
  if(m_edges.size() != (3 * *m_flockSize - 7))
  {
    std::cout<<"INDICES SIZE "<<m_edges.size()<<"\n";
    exit(EXIT_FAILURE);
  }
  int indicesSize2 = 2 * m_edges.size();
  int indices[indicesSize2];
  for(int i = 0; i < m_edges.size(); i++)
  {
    indices[2*i+0] = m_edges[i].m_boid_1->m_ID;
    indices[2*i+1] = m_edges[i].m_boid_2->m_ID;
  }

  edges->bind();
  edges->setNumIndices(indicesSize2);
  edges->setIndexedData(*m_flockSize*sizeof(Boid),m_boids[0].m_pos.m_x,indicesSize2,&indices[0],GL_UNSIGNED_INT,GL_STATIC_DRAW);
  edges->setVertexAttributePointer(0,3,GL_FLOAT,sizeof(Boid),0);
  edges->draw();
  edges->unbind();
}
