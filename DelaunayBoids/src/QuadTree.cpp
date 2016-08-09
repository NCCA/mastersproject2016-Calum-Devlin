#include <ngl/Transformation.h>
#include <ngl/VAOPrimitives.h>
#include <ngl/ShaderLib.h>

#include "include/QuadTree.h"

QuadTree::QuadTree(ngl::Vec3 _center, float _width, float _depth)
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
  isRoot = true;
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
    successor->~QuadTree();
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
    m_leaf = _boid;

    // This will give each boid a pointer
    // to the smallest QuadTree that completely contains it
    if(boidInQuad(m_leaf))
    {
      m_leaf->m_localRoot = this;
    }
  }
  // If this QuadTree already more than one child, pass _boid down
  else if(isParent)
  {
    // Place new Boid
    addBoidToQuadrant(_boid);
  }
  // Otherwise, we need to transition from single child to many successors.
  // Create 4 successors, and place 2 children, possibly recursively
  else
  {
    isParent = true;
    // Break leaf into 4 nodes
    ngl::Vec3 offset = ngl::Vec3(m_width,m_height,m_depth)/4.0f;
    //successorQuadTree = new QuadTree(m_center + offset, m_width/2,m_height/2, Quadrant::NE, this);
    m_successors.push_back(new QuadTree(m_center + offset, m_width/2, m_depth/2, Quadrant::NE, this));
    m_successors.push_back(new QuadTree(m_center - offset, m_width/2, m_depth/2, Quadrant::SW, this));

    offset = ngl::Vec3(-m_width,m_height,m_depth)/4.0f;
    m_successors.push_back(new QuadTree(m_center + offset, m_width/2, m_depth/2, Quadrant::NW, this));
    m_successors.push_back(new QuadTree(m_center - offset, m_width/2, m_depth/2, Quadrant::SE, this));

    // Place existing Boid
    addBoidToQuadrant(m_leaf);

    // Place new Boid
    addBoidToQuadrant(_boid);
  }
  m_allLeaves.push_back(_boid);
}
// LOOK INTO ONLY ADDING A NEW QUADRANT WHEN REQUIRED (during creation)

void QuadTree::addBoidToQuadrant(Boid *_boid)
{
  //bool successfulInsertion = false;
  for(auto &qt : m_successors)
  {
    // When successfully adding a new boid, skip testing the other quadrants
    qt->testBoid(_boid);
    /*if(successfulInsertion)
    {
      break;
    }*/
  }

  /*if(false)
  {
    ngl::Vec3 offset = ngl::Vec3(m_width,m_height,m_depth)/4.0f;
    if(_boid->m_pos.m_z > m_center.m_z)
    {
      if(_boid->m_pos.m_x > m_center.m_x)
      {
        m_successors.push_back(new QuadTree(m_center + offset, m_width/2, m_depth/2, Quadrant::NE, this));
      }
      else
      {
        offset = ngl::Vec3(-m_width,m_height,m_depth)/4.0f;
        m_successors.push_back(new QuadTree(m_center + offset, m_width/2, m_depth/2, Quadrant::NW, this));
      }
    }
    else
    {
      if(_boid->m_pos.m_x > m_center.m_x)
      {
        offset = ngl::Vec3(-m_width,m_height,m_depth)/4.0f;
        m_successors.push_back(new QuadTree(m_center - offset, m_width/2, m_depth/2, Quadrant::SE, this));
      }
      else
      {
        m_successors.push_back(new QuadTree(m_center - offset, m_width/2, m_depth/2, Quadrant::SW, this));
      }
    }
    m_successors.back()->addBoid(_boid);
  }*/
  /*requireNewQuadrant = !requireNewQuadrant;

  if(requireNewQuadrant)
  {
    bool transferQuadrant = false;
    for(auto &emptyQuadTree : m_emptySuccessors)
    {
      transferQuadrant = emptyQuadTree->testBoid(_boid);
      if(transferQuadrant == true)
      {
        m_successors.push_back(emptyQuadTree);
        std::swap(emptyQuadTree,m_emptySuccessors.end());
        m_emptySuccessors.swap();
        break;
      }
    }
    m_emptySuccessors.pop_back();
  }*/
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

void QuadTree::prune()
{
  if(isEmpty)
  {
    this->~QuadTree();
  }
}

/*************************************************************/
// The second IF statement can probably become an OR condition in the first
void QuadTree::think(Boid *_boid)
{
  // If the current QuadTree is completely inside the boid's radius,
  // We save time and skip to iterating over all leaves
  if(radiusContainsQuadTree(_boid))
  {
    for(auto &neighbour : m_allLeaves)
    {
      if(_boid != neighbour)
      {
        _boid->think(*neighbour);
      }
    }
  }
  // Edge case
  else if(!isEmpty && !isParent)
  {
    if(_boid != m_leaf)
    {
      _boid->think(*m_leaf);
    }
  }
  else
  {
    // Will only run when this Quadtree is not completely inside the radius, and has multiple children
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
  if((_boid->m_pos - ngl::Vec3(m_left,m_top,m_back)).length() > _boid->m_approachRadius)
  {
    return false;
  }
  if((_boid->m_pos - ngl::Vec3(m_left,m_top,m_front)).length() > _boid->m_approachRadius)
  {
    return false;
  }
  /*if((_boid->m_pos - ngl::Vec3(m_right,m_bottom,m_back)).length() > _boid->m_approachRadius)
  {
    return false;
  }
  if((_boid->m_pos - ngl::Vec3(m_right,m_bottom,m_front)).length() > _boid->m_approachRadius)
  {
    return false;
  }*/
  if((_boid->m_pos - ngl::Vec3(m_right,m_top,m_back)).length() > _boid->m_approachRadius)
  {
    return false;
  }
  if((_boid->m_pos - ngl::Vec3(m_right,m_top,m_front)).length() > _boid->m_approachRadius)
  {
    return false;
  }

  return true;
}

// If there is any overlap at all between boid and quadrant.
// The first test fails positive: some non-overlapping quadrants return true.
bool QuadTree::isNearQuadrant(Boid *_boid)
{
  // The diagonal length of the square, and the radius around the boid
  // are used to see if the quadrant is possibly inside the radius.
  // If so, the boids in the quadrant may be neighbours to consider.

  // Assume the container is a Square/Cube
  // for cube, use sqrt(3)
  float diagonal = sqrt(2)*m_width/2;
  float displacement = (_boid->m_pos - m_center).length();
  if((diagonal + _boid->m_approachRadius) > displacement)
  {
    //return true;
  }
  //return false;

  if(boidInQuad(_boid))
  {
    return true;
  }

  // If the distance between any corner and the boid is less than the boid radius, return true
  if((_boid->m_pos - ngl::Vec3(m_left,m_top,m_back)).length() < _boid->m_approachRadius)
  {
    return true;
  }
  if((_boid->m_pos - ngl::Vec3(m_left,m_top,m_front)).length() < _boid->m_approachRadius)
  {
    return true;
  }
  if((_boid->m_pos - ngl::Vec3(m_right,m_top,m_back)).length() < _boid->m_approachRadius)
  {
    return true;
  }
  if((_boid->m_pos - ngl::Vec3(m_right,m_top,m_front)).length() < _boid->m_approachRadius)
  {
    return true;
  }

  // However, this does not catch all cases, such as those where only
  // the middle of an edge intersects the boid radius.
  // The test takes the NSWE points on the boid radius,
  // and tests if they are inside the quadrant
  if(pointInQuad(_boid->m_pos - ngl::Vec3(_boid->m_approachRadius,0,0)))
  {
    return true;
  }
  if(pointInQuad(_boid->m_pos + ngl::Vec3(_boid->m_approachRadius,0,0)))
  {
    return true;
  }
  if(pointInQuad(_boid->m_pos - ngl::Vec3(0,0,_boid->m_approachRadius)))
  {
    return true;
  }
  if(pointInQuad(_boid->m_pos + ngl::Vec3(0,0,_boid->m_approachRadius)))
  {
    return true;
  }
  return false;

  // Octants will require an extra test: the edge/radius test becomes a face/radius test
  // and a new edge/radius test requires coding.
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

// Shorthand for 'Is the Boid and its radius of interest COMPLETELY inside this quadrant?'
// the first test fails negative: boids towards the corners of the quadrant can return false.
bool QuadTree::boidInQuad(Boid *_boid)
{
  float half_width = m_width/2;
  float displacement = (_boid->m_pos - m_center).length();
  if((displacement + _boid->m_approachRadius) < half_width)
  {
    //return true;
  }
  //return false;


  if((_boid->m_pos.m_x + _boid->m_approachRadius > m_right) || (_boid->m_pos.m_x - _boid->m_approachRadius < m_left))
  {
    return false;
  }
  /*if((_boid->m_pos.m_y + _boid->m_approachRadius > m_top) || (_boid->m_pos.m_y - _boid->m_approachRadius < m_bottom))
  {
    return false;
  }*/
  if((_boid->m_pos.m_z + _boid->m_approachRadius > m_front) || (_boid->m_pos.m_z - _boid->m_approachRadius < m_back))
  {
    return false;
  }
  return true;
}

void QuadTree::draw(const ngl::Mat4& _globalTransformationMatrix, ngl::Mat4 &_V, ngl::Mat4 &_VP)
{
  if(!isEmpty)
  {
    ngl::VAOPrimitives *prim = ngl::VAOPrimitives::instance();
    ngl::ShaderLib *shader = ngl::ShaderLib::instance();
    shader->use("nglDiffuseShader");

    ngl::Transformation transformation;
    transformation.setPosition(m_center.m_x,0+m_center.m_y,m_center.m_z);
    transformation.setScale(m_width,1,m_depth);

    ngl::Mat4 M = transformation.getMatrix() * _globalTransformationMatrix;
    ngl::Mat4 MV = M * _V;
    ngl::Mat4 MVP = MV * _VP;
    ngl::Mat3 normalMatrix = MV;
    normalMatrix.inverse();

    shader->setRegisteredUniform("M",M);
    shader->setRegisteredUniform("MV",MV);
    shader->setRegisteredUniform("MVP",MVP);
    shader->setRegisteredUniform("normalMatrix",normalMatrix);
    shader->setShaderParam4f("Colour",1,1,1,1);

    prim->draw("quadSquare");

    if(!isParent)
    {
      ngl::Colour boidColour = m_leaf->getColour();
      M = m_leaf->getTransformation() * _globalTransformationMatrix;
      MV = M * _V;
      MVP = MV * _VP;
      normalMatrix = MV;
      normalMatrix.inverse();

      shader->setRegisteredUniform("M",M);
      shader->setRegisteredUniform("MV",MV);
      shader->setRegisteredUniform("MVP",MVP);
      shader->setRegisteredUniform("normalMatrix",normalMatrix);
      shader->setShaderParam4f("Colour",boidColour.m_r,boidColour.m_g,boidColour.m_b,1);

      prim->draw("boid");
      /*if(m_leaf->m_ID == 1)
      {
        transformation.setPosition(m_leaf->m_pos.m_x,m_leaf->m_pos.m_y,m_leaf->m_pos.m_z);
        transformation.setScale(64,1,64);

        M = transformation.getMatrix() * _globalTransformationMatrix;
        MV = M * _V;
        MVP = MV * _VP;
        normalMatrix = MV;
        normalMatrix.inverse();

        shader->setRegisteredUniform("M",M);
        shader->setRegisteredUniform("MV",MV);
        shader->setRegisteredUniform("MVP",MVP);
        shader->setRegisteredUniform("normalMatrix",normalMatrix);
        shader->setShaderParam4f("Colour",1,0,0,1);

        prim->draw("quadSquare");
      }*/
    }

    for(auto &child : m_successors)
    {
      child->draw(_globalTransformationMatrix, _V, _VP);
    }
  }
}
