#ifndef QUADTREE_H
#define QUADTREE_H

#include <vector>
#include <ngl/Vec3.h>
#include "Boid.h"

/******************************************************************************
 * Much commented out code allows the potential for an upgrade to 3D octants. *
 * But some code has not been written yet, so beware of gaps that allow bugs. *
 ******************************************************************************/

// Letters are pairwise: Up/Down; North/South; West/East
enum class Quadrant
{
  ROOT, NE, NW, SE, SW
};

enum class Octant
{
  ROOT, UNW, UNE, USW, USE, DNW, DNE, DSW, DSE
};

class QuadTree
{
  public:
    QuadTree(ngl::Vec3 _center, float _width/*, float _height*/, float _depth);
    QuadTree(ngl::Vec3 _center, float _width/*, float _height*/, float _depth, Quadrant _quadrantType, QuadTree *_parent);
    ~QuadTree();

    bool isRoot = false;
    bool isParent;  // To distinguish between leaves and nodes
    bool isEmpty;

    void setDefaultLocalRoot(Boid *_boid);

    void think(Boid *_boid);
    void draw(const ngl::Mat4& _globalTransformationMatrix, ngl::Mat4 &_V, ngl::Mat4 &_VP);

  private:
    /*const*/ QuadTree *m_parent;
    QuadTree *m_localRoot;
    Quadrant m_quadrantType;
    Octant m_octantType;

    // Camera Pointer
    ngl::Camera *m_cam;
    // Shader Reference
    std::string m_shaderName;
    ngl::Mat4 V;
    ngl::Mat4 VP;

    ngl::Vec3 m_center;
    float m_width;
    float m_height;
    float m_depth;
    float m_left;
    float m_right;
    float m_top;
    float m_bottom;
    float m_front;
    float m_back;

    Boid *m_leaf;
    std::vector<Boid*> m_allLeaves;

    void addBoid(Boid *_boid);
    void addBoid2(Boid _boid);
    void addBoid3(Boid &_boid);
    void addBoidToQuadrant(Boid *_boid);
    void testBoid(Boid *_boid);
    void prune();

    Quadrant getOpposingQuadrant(Quadrant _q);

    bool radiusContainsQuadTree(Boid *_boid);
    bool isNearQuadrant(Boid *_boid);
    bool boidInQuad(Boid *_boid);
    bool pointInQuad(ngl::Vec3 _point);
    Quadrant pointInQuad2(ngl::Vec3 _point);

    QuadTree *NE;
    QuadTree *NW;
    QuadTree *SE;
    QuadTree *SW;

    std::vector<QuadTree*> m_successors;
    //std::vector<QuadTree*> m_emptySuccessors;
};

#endif // QUADTREE_H
