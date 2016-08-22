#include <ngl/Transformation.h>
#include <ngl/VAOPrimitives.h>
#include <ngl/ShaderLib.h>

#include "include/QuadTree.h"

QuadTree::QuadTree(ngl::Vec3 _center, float _width/*, float _height*/, float _depth)
{
  isParent = false;
  isEmpty = true;

  m_center = _center;
  m_width = _width;
  //m_height = _height;
  m_height = 0;
  m_depth = _depth;

  m_left =    m_center.m_x - (m_width/2);
  m_right =   m_center.m_x + (m_width/2);

  m_bottom =  m_center.m_y - (m_height/2);
  m_top =     m_center.m_y + (m_height/2);

  m_back =    m_center.m_z - (m_depth/2);
  m_front =   m_center.m_z + (m_depth/2);

  m_quadrantType = Quadrant::ROOT;
}

QuadTree::QuadTree(ngl::Vec3 _center, float _width/*, float _height*/, float _depth, Quadrant _quadrantType, QuadTree *_parent)
{
  isParent = false;
  isEmpty = true;

  m_center = _center;
  m_width = _width;
  //m_height = _height;
  m_height = 0;
  m_depth = _depth;

  m_left =    m_center.m_x - (m_width/2);
  m_right =   m_center.m_x + (m_width/2);
  m_bottom =  m_center.m_y - (m_height/2);
  m_top =     m_center.m_y + (m_height/2);
  m_back =    m_center.m_z - (m_depth/2);
  m_front =   m_center.m_z + (m_depth/2);

  m_parent = _parent;
  m_quadrantType = _quadrantType;
}

QuadTree::~QuadTree()
{
  for(auto &successor : m_successors)
  {
    delete successor;
    //successor->~QuadTree();
  }
}

// This is a method exclusive to the root node, to ensure every boid is guaranteed a QuadTree that completely contains it.
void QuadTree::setDefaultLocalRoot(Boid *_boid)
{
  _boid->m_localRoot = this;
  addBoid(_boid);
}

/*************************************************************/
void QuadTree::addBoid(Boid *_boid)
{
  // If this QuadTree has no children, we assign one
  if(isEmpty)
  {
    isEmpty = false;

    // This will recursively give each boid a pointer to the smallest QuadTree that completely contains it
    if(boidInQuad(_boid))
    {
      _boid->m_localRoot = this;
    }
  }
  // If this QuadTree already has more than one child, pass _boid down
  else if(isParent)
  {
    // Place new Boid
    addBoidToQuadrant(_boid);
  }
  // Otherwise, we need to transition from single child to many children.
  // Create 4 successors, and place 2 boid pointers, possibly recursively
  else
  {
    isParent = true;
    // Break leaf into 4 nodes
    ngl::Vec3 offset1 = ngl::Vec3(m_width,m_height,m_depth)/4.0f;
    m_successors.push_back(new QuadTree(m_center + offset1, m_width/2, m_depth/2, Quadrant::NE, this));
    m_successors.push_back(new QuadTree(m_center - offset1, m_width/2, m_depth/2, Quadrant::SW, this));

    ngl::Vec3 offset2 = ngl::Vec3(-m_width,m_height,m_depth)/4.0f;
    m_successors.push_back(new QuadTree(m_center + offset2, m_width/2, m_depth/2, Quadrant::NW, this));
    m_successors.push_back(new QuadTree(m_center - offset2, m_width/2, m_depth/2, Quadrant::SE, this));

    // Place existing Boid
    addBoidToQuadrant(m_allLeaves[0]);

    // Place new Boid
    addBoidToQuadrant(_boid);
  }
  m_allLeaves.push_back(_boid);
}
// LOOK INTO ONLY ADDING A NEW QUADRANT WHEN REQUIRED (during creation)

void QuadTree::addBoidToQuadrant(Boid *_boid)
{
  for(auto &qt : m_successors)
  {
    // Potential refinement: After successfully adding a new boid, skip testing the other quadrants
    qt->testBoid(_boid);
  }
}

void QuadTree::testBoid(Boid *_boid)
{
  // The mathematics and logic would be simpler if handled by the parent,
  // but would require hardcoding which child receives the boid
  if(((m_left   <= (_boid->m_pos.m_x))
   && (m_right  >  (_boid->m_pos.m_x)))
   //&&((m_bottom <= (_boid->m_pos.m_y))
   //&& (m_top    >  (_boid->m_pos.m_y)))
   &&((m_back   <= (_boid->m_pos.m_z))
   && (m_front  >  (_boid->m_pos.m_z))))
  {
    addBoid(_boid);
  }
}

/*************************************************************/
void QuadTree::think(Boid *_boid)
{
  // If the current QuadTree is completely inside the boid's radius,
  // save time and skip to iterating over all leaves.
  // We also include the terminal step as a condition because m_allLeaves will only contain 1 leaf
  if(radiusContainsQuadTree(_boid) || (!isEmpty && !isParent))
  {
    for(auto &neighbour : m_allLeaves)
    {
      _boid->think(*neighbour);
    }
  }
  else
  {
    // This recursion will only execute when:
    // This QuadTree is not completely inside the radius
    // This QuadTree has multiple children
    if(isNearQuadrant(_boid))
    {
      for(auto &successor : m_successors)
      {
        if(!successor->isEmpty)
        {
          successor->think(_boid);
        }
      }
    }
  }
}

// Returns true if the corners of the QuadTree are completely inside the radius of the given boid.
bool QuadTree::radiusContainsQuadTree(Boid *_boid)
{
  /*if((_boid->m_pos - ngl::Vec3(m_left,m_bottom,m_back)).length() > _boid->m_approachRadius)
  {
    return false;
  }
  if((_boid->m_pos - ngl::Vec3(m_left,m_bottom,m_front)).length() > _boid->m_approachRadius)
  {
    return false;
  }*/
  if((_boid->m_pos - ngl::Vec3(m_left,m_top,m_back)).length() > *_boid->m_approachRadius)
  {
    return false;
  }
  if((_boid->m_pos - ngl::Vec3(m_left,m_top,m_front)).length() > *_boid->m_approachRadius)
  {
    return false;
  }
  /*if((_boid->m_pos - ngl::Vec3(m_right,m_bottom,m_back)).length() > *_boid->m_approachRadius)
  {
    return false;
  }
  if((_boid->m_pos - ngl::Vec3(m_right,m_bottom,m_front)).length() > *_boid->m_approachRadius)
  {
    return false;
  }*/
  if((_boid->m_pos - ngl::Vec3(m_right,m_top,m_back)).length() > *_boid->m_approachRadius)
  {
    return false;
  }
  if((_boid->m_pos - ngl::Vec3(m_right,m_top,m_front)).length() > *_boid->m_approachRadius)
  {
    return false;
  }

  return true;
}

// If there is any overlap at all between boid radius and quadrant.
// The first test fails positive: some non-overlapping quadrants return true.
bool QuadTree::isNearQuadrant(Boid *_boid)
{
  /*******************************
   * FIRST TEST; APPROXIMATION.  *
   *******************************/

  // The diagonal length of the square, and the radius around the boid
  // are used to see if the quadrant is possibly inside the radius.
  // If so, the boids in the quadrant may be neighbours to consider.

  // Assume the container is a Square/Cube
  float quadrantDiagonal = sqrt(2)*m_width/2;
  //float octantDiagonal = sqrt(3)*m_width/2;
  float displacement = (_boid->m_pos - m_center).length();
  if((quadrantDiagonal + *_boid->m_approachRadius) > displacement)
  {
    //return true;
  }
  //return false;


  /*******************************
   * SECOND TEST; EXACT.         *
   *******************************/
  if(boidInQuad(_boid))
  {
    return true;
  }

  // If the distance between any corner and the boid is less than the boid radius, return true
  if((_boid->m_pos - ngl::Vec3(m_left,m_top,m_back)).length() < *_boid->m_approachRadius)
  {
    return true;
  }
  if((_boid->m_pos - ngl::Vec3(m_left,m_top,m_front)).length() < *_boid->m_approachRadius)
  {
    return true;
  }
  if((_boid->m_pos - ngl::Vec3(m_right,m_top,m_back)).length() < *_boid->m_approachRadius)
  {
    return true;
  }
  if((_boid->m_pos - ngl::Vec3(m_right,m_top,m_front)).length() < *_boid->m_approachRadius)
  {
    return true;
  }

  // However, this does not catch all cases, such as those where only
  // the middle of an edge intersects the boid radius.
  // The test takes the NSWE points on the boid radius,
  // and tests if they are inside the quadrant
  if(pointInQuad(_boid->m_pos - ngl::Vec3(*_boid->m_approachRadius,0,0)))
  {
    return true;
  }
  if(pointInQuad(_boid->m_pos + ngl::Vec3(*_boid->m_approachRadius,0,0)))
  {
    return true;
  }
  if(pointInQuad(_boid->m_pos - ngl::Vec3(0,0,*_boid->m_approachRadius)))
  {
    return true;
  }
  if(pointInQuad(_boid->m_pos + ngl::Vec3(0,0,*_boid->m_approachRadius)))
  {
    return true;
  }
  return false;

  // Octants will require an extra test: the edge/radius test becomes a face/radius test
  // and a new edge/radius test requires coding.
  // As an intuitive start, project the sphere & cube down a dimension, and use the old E/R test, and check boid height
}

bool QuadTree::pointInQuad(ngl::Vec3 _point)
{
  if((_point.m_x >= m_left && _point.m_x <= m_right)
     //&& (_point.m_y >= m_bottom && _point.m_y <= m_top)
     && (_point.m_z >= m_back && _point.m_z <= m_front))
  {
    return true;
  }
  return false;
}

Quadrant QuadTree::pointInQuad2(ngl::Vec3 _point)
{
  for(auto &successor : m_successors)
  {
    if((_point.m_x >= successor->m_left && _point.m_x <= successor->m_right)
       //&& (_point.m_y >= successor->m_bottom && _point.m_y <= successor->m_top)
       && (_point.m_z >= successor->m_back && _point.m_z <= successor->m_front))
    {
      return m_quadrantType;
    }
  }
  // Successor nodes are never ROOT type, so this is acceptable as a default return value.
  return Quadrant::ROOT;
}

// Shorthand for 'Is the Boid and its radius of interest COMPLETELY inside this quadrant?'
// the first test fails negative: boids towards the corners of the quadrant can return false.
bool QuadTree::boidInQuad(Boid *_boid)
{
  /*******************************
   * FIRST TEST; APPROXIMATION.  *
   *******************************/
  float half_width = m_width/2;
  float displacement = (_boid->m_pos - m_center).length();
  if((displacement + *_boid->m_approachRadius) < half_width)
  {
    //return true;
  }
  //return false;


  /*******************************
   * SECOND TEST; EXACT.         *
   *******************************/
  if((_boid->m_pos.m_x + *_boid->m_approachRadius > m_right) || (_boid->m_pos.m_x - *_boid->m_approachRadius < m_left))
  {
    return false;
  }
  /*if((_boid->m_pos.m_y + *_boid->m_approachRadius > m_top) || (_boid->m_pos.m_y - *_boid->m_approachRadius < m_bottom))
  {
    return false;
  }*/
  if((_boid->m_pos.m_z + *_boid->m_approachRadius > m_front) || (_boid->m_pos.m_z - *_boid->m_approachRadius < m_back))
  {
    return false;
  }
  return true;
}

void QuadTree::draw(const ngl::Mat4& _globalTransformationUnderCamera, int _inspectIndex)
{
  if(!isEmpty)
  {
    ngl::VAOPrimitives *prim = ngl::VAOPrimitives::instance();
    ngl::ShaderLib *shader = ngl::ShaderLib::instance();
    shader->use("nglDiffuseShader");

    ngl::Transformation transformation;
    transformation.setPosition(m_center.m_x,0+m_center.m_y,m_center.m_z);
    transformation.setScale(m_width,1,m_depth);

    ngl::Mat4 MVP = transformation.getMatrix() * _globalTransformationUnderCamera;

    shader->setRegisteredUniform("MVP",MVP);
    shader->setShaderParam4f("Colour",1,1,1,1);

    prim->draw("quadSquare");

    if(!isParent && (m_allLeaves[0]->m_ID == _inspectIndex))
    {
      transformation.setPosition(m_allLeaves[0]->m_pos.m_x,m_allLeaves[0]->m_pos.m_y,m_allLeaves[0]->m_pos.m_z);
      transformation.setScale(*m_allLeaves[0]->m_approachRadius,1,*m_allLeaves[0]->m_approachRadius);

      MVP = transformation.getMatrix() * _globalTransformationUnderCamera;

      shader->setRegisteredUniform("MVP",MVP);
      shader->setShaderParam4f("Colour",1,0,0,1);

      prim->draw("quadSquare");
    }
    else
    {
      for(auto &child : m_successors)
      {
        child->draw(_globalTransformationUnderCamera, _inspectIndex);
      }
    }
  }
}
