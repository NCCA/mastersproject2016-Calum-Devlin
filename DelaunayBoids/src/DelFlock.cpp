#include <ngl/Transformation.h>
#include <ngl/VAOPrimitives.h>
#include <ngl/ShaderLib.h>
#include <ngl/Random.h>

#include "DelFlock.h"

DelFlock::DelFlock(const int *_flockSize, float _boidRadius, ngl::Vec3 _flockOrigin)
  : Flock(_flockSize, _boidRadius, _flockOrigin)
{
  edges.reset(ngl::VertexArrayObject::createVOA(GL_LINES));

  //debug();
  // Construct the DG

  /*GLfloat *bp = new GLfloat[1000*3];
  for(int i = 0; i < 1000; i++)
  {
    bp[0+3*i] = m_boids[i].m_pos.m_x;
    bp[1+3*i] = m_boids[i].m_pos.m_y;
    bp[2+3*i] = m_boids[i].m_pos.m_z;
  }
  float bpp = m_boids[0].m_pos.m_x;
  //float *a = *m_boids[0].m_pos.m_x;
  //float b = *m_boids[0].m_pos.m_x;
  //float &c = *m_boids[0].m_pos.m_x;

  //float *d = m_boids[0].m_pos.m_x;
  float e = m_boids[0].m_pos.m_x;
  float &f = m_boids[0].m_pos.m_x;

  float *g = &m_boids[0].m_pos.m_x;
  //float h = &m_boids[0].m_pos.m_x;
  //float &i = &m_boids[0].m_pos.m_x;

  Boid &flockPointer = *m_boids.begin();

  float z = sizeof(Boid);
  float z2 = sizeof(Boid);*/

  // Borrow the first four boids to pin the boundary of the delaunay graph
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
        int interferingBoids = 0;
        int leftBoids = 0;
        int rightBoids = 0;
        ngl::Vec3 mid_pos = (boidA.m_pos + boidB.m_pos)/2;
        float potentialDelaunayEdgeRadius = (boidA.m_pos - mid_pos).length();
        DelaunayTriangle PDTleft = DelaunayTriangle();// Potential Delaunay Traingle
        DelaunayTriangle PDTright = DelaunayTriangle();// Potential Delaunay Traingle

        for(auto &boidC : m_boids)
        {
          if((&boidA.m_ID != &boidC.m_ID)
             && (&boidB.m_ID != &boidC.m_ID))
          {
            bool leftVSright = isLeft(&boidA.m_pos, &boidB.m_pos, &boidC.m_pos);
            if(leftVSright)
            {
              // If it's the first boid encountered on this side, make a new delaunay triangle
              if(leftBoids == 0)
              {
                PDTleft = DelaunayTriangle(&boidA, &boidB, &boidC);
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
                  PDTleft = DelaunayTriangle(&boidA, &boidB, &boidC);
                }
              }
              // In either case, check if this boid invalidates the delaunay triangle on the other side
              float circumRadius = (PDTright.m_circumcenter - boidA.m_pos).length();
              float circumCenterDistance = (PDTright.m_circumcenter - boidC.m_pos).length();
              if(circumCenterDistance < circumRadius)
              {
                //PDTright = DelaunayTriangle();
                //rightBoids = 0;
              }
            }
            else
            {
              // If it's the first boid encountered on this side, make a new delaunay triangle
              if(rightBoids == 0)
              {
                PDTright = DelaunayTriangle(&boidA, &boidB, &boidC);
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
                  PDTright = DelaunayTriangle(&boidA, &boidB, &boidC);
                }
              }
              // In either case, check if this boid invalidates the delaunay triangle on the other side
              float circumRadius = (PDTleft.m_circumcenter - boidA.m_pos).length();
              float circumCenterDistance = (PDTleft.m_circumcenter - boidC.m_pos).length();
              if(circumCenterDistance < circumRadius)
              {
                //PDTleft = DelaunayTriangle();
                //leftBoids = 0;
              }
            }



            /*if(interferingBoids == 0)
            {
              PDTleft = DelaunayTriangle(&boidA, &boidB, &boidC);
              ++interferingBoids;
            }
            else
            {
              float circumRadius = (PDTleft.m_circumcenter - boidA.m_pos).length();
              float circumCenterDistance = (PDTleft.m_circumcenter - boidC.m_pos).length();
              if(circumCenterDistance < circumRadius)
              {
                PDTleft = DelaunayTriangle(&boidA, &boidB, &boidC);
              }
            }*/


            /*float neighbourDistance = (boidC.m_pos - mid_pos).length();
            if(neighbourDistance < potentialDelaunayEdgeRadius)
            {
              ++interferingBoids;
              potentialDelaunayEdgeRadius = neighbourDistance;
              if(interferingBoids == 1)
              {
                DelaunayTriangle dt = DelaunayTriangle(&boidA, &boidB, &boidC);
              }
            }*/
          }
        }
        if(interferingBoids < 1)
        {
          // Only creates a Gabriel Graph
          DelaunayEdge de = DelaunayEdge(&boidA, &boidB);
          //m_edges.push_back(de);
        }

        //bool validPair = false;
        if(PDTleft.m_exists && PDTright.m_exists)
        {
          float circumRadiusLeft = (PDTleft.m_circumcenter - boidA.m_pos).length();
          float circumCenterDistanceLeft = (PDTleft.m_circumcenter - PDTright.b3->m_pos).length();
          float circumRadiusRight = (PDTright.m_circumcenter - boidA.m_pos).length();
          float circumCenterDistanceRight = (PDTright.m_circumcenter - PDTleft.b3->m_pos).length();
          if((circumCenterDistanceLeft > circumRadiusLeft)
             && (circumCenterDistanceRight > circumRadiusRight))
            //if((PDTleft.b3->m_pos - PDTright.b3->m_pos).length() > (boidA.m_pos - boidB.m_pos).length())
          {
            DelaunayEdge de1 = DelaunayEdge(PDTleft.b1, PDTleft.b2);
            //m_edges.push_back(de1);

            DelaunayEdge de2 = DelaunayEdge(PDTleft.b1, PDTleft.b3);
            m_edges.push_back(de2);
            DelaunayEdge de3 = DelaunayEdge(PDTleft.b3, PDTleft.b2);
            m_edges.push_back(de3);

            DelaunayEdge de6 = DelaunayEdge(PDTright.b1, PDTright.b2);
            //m_edges.push_back(de6);
            DelaunayEdge de4 = DelaunayEdge(PDTright.b1, PDTright.b3);
            m_edges.push_back(de4);
            DelaunayEdge de5 = DelaunayEdge(PDTright.b3, PDTright.b2);
            m_edges.push_back(de5);
          }
        }
        /*else
        {
          // Four boundary cases
          if(PDTleft.m_exists)
          {
            DelaunayEdge de1 = DelaunayEdge(PDTleft.b1, PDTleft.b2);
            m_edges.push_back(de1);
            DelaunayEdge de2 = DelaunayEdge(PDTleft.b1, PDTleft.b3);
            m_edges.push_back(de2);
            DelaunayEdge de3 = DelaunayEdge(PDTleft.b3, PDTleft.b2);
            m_edges.push_back(de3);
          }
          if(PDTright.m_exists)
          {
            DelaunayEdge de1 = DelaunayEdge(PDTright.b1, PDTright.b2);
            m_edges.push_back(de1);
            DelaunayEdge de2 = DelaunayEdge(PDTright.b1, PDTright.b3);
            m_edges.push_back(de2);
            DelaunayEdge de3 = DelaunayEdge(PDTright.b3, PDTright.b2);
            m_edges.push_back(de3);
          }
        }*/
      }
    }
  }


  /*for(auto &boidA : m_boids)
  {
    for(auto &boidB : m_boids)
    {
      if(&boidA.m_ID > &boidB.m_ID)
      {
        for(auto &boidC : m_boids)
        {
          if(&boidB.m_ID > &boidC.m_ID)
          {
            int interferingBoids = 0;
            for(auto &boidD : m_boids)
            {
              if((&boidA.m_ID != &boidD.m_ID)
                 && (&boidB.m_ID != &boidD.m_ID)
                 && (&boidC.m_ID != &boidD.m_ID))
              {
                ngl::Vec3 mid_pos = (boidA.m_pos + boidB.m_pos + boidC.m_pos)/3;
                float potentialDelaunayEdgeRadius = (boidA.m_pos - mid_pos).length();
                float potentialDelaunayEdgeRadius2 = (boidB.m_pos - mid_pos).length();
                float potentialDelaunayEdgeRadius3 = (boidC.m_pos - mid_pos).length();
                ++interferingBoids;
              }
            }
            if(interferingBoids < 1)
            {
              DelaunayEdge de1 = DelaunayEdge(&boidA, &boidB);
              DelaunayEdge de2 = DelaunayEdge(&boidB, &boidC);
              DelaunayEdge de3 = DelaunayEdge(&boidC, &boidA);
              //m_edges.push_back(de1);
              //m_edges.push_back(de2);
              //m_edges.push_back(de3);
            }
          }
        }
      }
    }
  }*/

  /*edges->bind();
  //edges->setData(4*sizeof(Boid),data[0].m_pos.m_x);
  edges->setIndexedData(1000*sizeof(Boid),m_boids[0].m_pos.m_x,1998,&indices[0],GL_UNSIGNED_INT,GL_STATIC_DRAW);

  edges->setVertexAttributePointer(0,3,GL_FLOAT,sizeof(Boid),0);
  edges->unbind();*/
}

DelFlock::~DelFlock()
{
  m_boids.clear();
  m_edges.clear();
}

void DelFlock::think()
{
  moveTarget();
  m_target = ngl::Vec3(750*sin(theta),0,750*cos(theta));

  for(auto &boid : m_boids)
  {
    boid.clear();
    for(auto &neighbour : m_boids)
    {
      boid.think(neighbour);
    }
  }

  for(auto &edge : m_edges)
  {
    edge.doubleBoidThink();
  }

  for(auto &boid : m_boids)
  {
    boid.move(m_target);
  }

  for(auto &edge : m_edges)
  {
    // Test if the edge can flip.  I am ... trusting a paper I have seen that says (paraphrased):
    // Either an edge is locally delaunay, or it's flip is
    // I'm almost certain this means I only have to test each edge once, and get his done in linear time.
    // Best way to handle this: like quadtrees, replace the vector every time, and have this test return an edge every time.
    // Bugger, I got that wrong.  But the paper goes on to say that minor jitter should be fast, so...

  }
}

void DelFlock::draw(const ngl::Mat4& _globalTransformationMatrix) const
{
  ngl::VAOPrimitives *prim = ngl::VAOPrimitives::instance();
  ngl::ShaderLib *shader = ngl::ShaderLib::instance();
  shader->use("nglDiffuseShader");

  ngl::Mat4 V = getCam()->getViewMatrix();
  ngl::Mat4 VP = getCam()->getVPMatrix();

  for(Boid b : m_boids)
  {
    if(b.m_canMove)
    {
      ngl::Mat4 MVP = b.getTransformation() * _globalTransformationMatrix * V * VP;
      shader->setRegisteredUniform("MVP",MVP);

      ngl::Colour boidColour = b.getColour();
      shader->setShaderParam4f("Colour",boidColour.m_r,boidColour.m_g,boidColour.m_b,1);

      prim->draw("boid");
    }
  }

  // Also draw the target that the boids are following
  ngl::Transformation transformation;
  transformation.setPosition(m_target.m_x,m_target.m_y,m_target.m_z);
  ngl::Mat4 MVP = transformation.getMatrix() * _globalTransformationMatrix * V * VP;
  shader->setRegisteredUniform("MVP",MVP);
  shader->setShaderParam4f("Colour",1,0,0,1);

  prim->draw("target");

  MVP = _globalTransformationMatrix * V * VP;
  shader->setRegisteredUniform("MVP",MVP);

  shader->setShaderParam4f("Colour",0,0,0,1);

  // DRAW THE EDGES

  /*int delaunayIndices[m_delaunayEdges];
  int noInternalEdges = m_delaunayEdges;
  for(int i = 0; i < 1000; i++)
  {
    Boid b = *m_edges[0].m_boid_1;
    Boid b2 = *m_edges[0].m_boid_2;
    if(b.m_canMove && b2.m_canMove)
    {
      delaunayIndices[2*i+0] = b.getID();
      delaunayIndices[2*i+1] = b2.m_ID;
      --noInternalEdges;
    }
  }
  for(int i = 0; i < 1000;)
  {
    Boid b = *m_edges[0].m_boid_1;
    Boid b2 = *m_edges[0].m_boid_2;
    if(b.m_canMove && b2.m_canMove)
    {
      delaunayIndices[++i] = b.m_ID;
      delaunayIndices[++i] = b2.m_ID;
      --noInternalEdges;
    }
  }*/



  ngl::Random *RNG = ngl::Random::instance();
  /* int indicesSize = 2 * *m_flockSize - 8;
   * int indices[indicesSize];
  for(int i = 4; i < *m_flockSize; i++)
  {
    indices[(2*i)-8] = 4;//RNG->randomPositiveNumber(*m_flockSize);
    indices[(2*i)-7] = i;//RNG->randomPositiveNumber(*m_flockSize);
  }*/

  //int indicesSize = *m_flockSize * *m_flockSize ;
  //int indicesSize2 = 2 * *m_flockSize * *m_flockSize ;
  int indicesSize = m_edges.size();
  int indicesSize2 = 2 * indicesSize;
  int indices[indicesSize2];
  for(int i = 0; i < indicesSize; i++)
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

bool DelFlock::isLeft(ngl::Vec3 *_from, ngl::Vec3 *_to, ngl::Vec3 *_side)
{
  ngl::Real a = (_to->m_x - _from->m_x)*(_side->m_z - _from->m_z);
  ngl::Real b = (_to->m_z - _from->m_z)*(_side->m_x - _from->m_x);
  if(a==b)
  {
    std::cout<<"BEWARE THE LINEAR CASE";
  }
  return (a > b);
}
