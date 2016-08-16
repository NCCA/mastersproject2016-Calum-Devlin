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
    //m_leaf = _boid;

    // This will give each boid a pointer
    // to the smallest QuadTree that completely contains it
    //if(boidInQuad(m_leaf))
    if(boidInQuad(_boid))
    {
      //m_leaf->m_localRoot = this;
      _boid->m_localRoot = this;
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
    ngl::Vec3 offset1 = ngl::Vec3(m_width,m_height,m_depth)/4.0f;
    m_successors.push_back(new QuadTree(m_center + offset1, m_width/2, m_depth/2, Quadrant::NE, this));
    m_successors.push_back(new QuadTree(m_center - offset1, m_width/2, m_depth/2, Quadrant::SW, this));

    ngl::Vec3 offset2 = ngl::Vec3(-m_width,m_height,m_depth)/4.0f;
    m_successors.push_back(new QuadTree(m_center + offset2, m_width/2, m_depth/2, Quadrant::NW, this));
    m_successors.push_back(new QuadTree(m_center - offset2, m_width/2, m_depth/2, Quadrant::SE, this));

    // Place existing Boid
    //addBoidToQuadrant(m_leaf);
    addBoidToQuadrant(m_allLeaves[0]);

    // Place new Boid
    addBoidToQuadrant(_boid);
  }
  m_allLeaves.push_back(_boid);
}
// LOOK INTO ONLY ADDING A NEW QUADRANT WHEN REQUIRED (during creation)

void QuadTree::addBoid2(Boid _boid)
{
  // If this QuadTree has neither child nor successors,
  // we create a child
  if(isEmpty)
  {
    isEmpty = false;
    m_leaf = &_boid;
  }
  // If this QuadTree already has successors, pass *_boid down
  else if(isParent)
  {
    // Place new Boid
    addBoidToQuadrant(&_boid);
  }
  // Otherwise, we need to transition from single child to many successors.
  // Create 4 successors, and place 2 children, possibly recursive
  else
  {
    isParent = true;
    // Break leaf into 4 nodes
    ngl::Vec3 offset = ngl::Vec3(m_width,0,m_height)/4.0f;
    NE = new QuadTree(m_center + offset, m_width/2,m_height/2, Quadrant::NE, this);
    SW = new QuadTree(m_center - offset, m_width/2,m_height/2, Quadrant::SW, this);

    offset = offset = (-m_width,0,m_height)/4.0f;
    NW = new QuadTree(m_center + offset, m_width/2,m_height/2, Quadrant::NW, this);
    SE = new QuadTree(m_center - offset, m_width/2,m_height/2, Quadrant::SE, this);

    // Place existing Boid
    addBoidToQuadrant(m_leaf);

    // Place new Boid
    addBoidToQuadrant(&_boid);
  }
}

void QuadTree::addBoid3(Boid &_boid)
{
  // If this QuadTree has neither child nor successors,
  // we create a child
  if(isEmpty)
  {
    isEmpty = false;
    m_leaf = &_boid;
  }
  // If this QuadTree already has successors, pass *_boid down
  else if(isParent)
  {
    // Place new Boid
    addBoidToQuadrant(&_boid);
  }
  // Otherwise, we need to transition from single child to many successors.
  // Create 4 successors, and place 2 children, possibly recursive
  else
  {
    isParent = true;
    // Break leaf into 4 nodes
    ngl::Vec3 offset = ngl::Vec3(m_width,0,m_height)/4.0f;
    NE = new QuadTree(m_center + offset, m_width/2,m_height/2, Quadrant::NE, this);
    SW = new QuadTree(m_center - offset, m_width/2,m_height/2, Quadrant::SW, this);

    offset = offset = (-m_width,0,m_height)/4.0f;
    NW = new QuadTree(m_center + offset, m_width/2,m_height/2, Quadrant::NW, this);
    SE = new QuadTree(m_center - offset, m_width/2,m_height/2, Quadrant::SE, this);

    // Place existing Boid
    addBoidToQuadrant(m_leaf);

    // Place new Boid
    addBoidToQuadrant(&_boid);
  }
}

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

Quadrant QuadTree::getOpposingQuadrant(Quadrant _q)
{
  /*switch(_q)
  {
    case Quadrant::NE : return Quadrant::SW;
    case Quadrant::NW : return Quadrant::SE;
    case Quadrant::SW : return Quadrant::NE;
    case Quadrant::SE : return Quadrant::NW;
  }*/
  return Quadrant::ROOT;
}

/*************************************************************/
void QuadTree::think(Boid *_boid)
{
  /*
  Step 1:
  Boid is completely in Quadrant, and in at least 2 subquadrants
  If center point is within radius, then boid is in 4 subquadrants
  Otherwise, identify the subquadrant, and choose +/- 1 of quadtype (mod 4) for a little speed

  Step 2:
  Neither boid nor quadrant completely contain each other
  boid may be in 1-4 subquadrants
  Again, if center point is in radius, evaluate all 4 subquadrants

  Step 3:
  By this iteration, it is possible for the boid to contain the quadrant.
  If so, then all three siblings need reiteration

  Step 4:
  We could now have 0-3 subquadrants completely contained in the boid - not 4, as that would be caught in step 3
  */

  // Terminal condition: if we have reached a leaf node, compare the vector of leaves (length 1).
  /*if(!isEmpty && !isParent)
  {
    for(auto &neighbour : m_allLeaves)
    {
      _boid->think(*neighbour);
    }
  }
  // Pass down to the children nodes: either ignore, execute recursively, or take a shortcut.
  else
  {
    // Calculate which quadrant the boid is in, and its relative displacement to this quadrant
    float difference = (m_center - _boid->m_pos).length();
    Quadrant boidQuadrant = pointInQuad2(_boid->m_pos);

    if(difference < _boid->m_approachRadius)
    {
      // All children overlap the boid radius, so all need evaluating
      for(auto &successor : m_successors)
      {
        if(!successor->isEmpty)
        {
					//if(successor->m_quadrantType == boidQuadrant)
          {
            // If this successor QuadTree is completely inside the boid's radius,
            // We save time and skip to iterating over all leaves
						if(successor->radiusContainsQuadTree(_boid))
            {
              for(auto &neighbour : successor->m_allLeaves)
              {
                _boid->think(*neighbour);
              }
						}
            else
            {
              successor->think(_boid);
						}
          }
					//else
					{
						//successor->think(_boid);
          }
        }
      }
    }
    else
    {
      // This entire quadrant may be too far away from the boid -> all successors are too far away also.
			//if(boidQuadrant != Quadrant::ROOT)
      {
        // This quadrant's centre point is too far away from the boid, so at least 1 quadrant can be skipped
        // I.E. The quadrant opposite the one containing the boid
        boidQuadrant = getOpposingQuadrant(boidQuadrant);

        for(auto &successor : m_successors)
        {
          if(!successor->isEmpty)
          {
						if(successor->m_quadrantType != boidQuadrant)
            {
              successor->think(_boid);
            }
          }
        }
      }
    }
	}
  return;*/

  // If the current QuadTree is completely inside the boid's radius,
  // We save time and skip to iterating over all leaves
  // We also include the terminal step as a condition because m_allLeaves will only contain 1 leaf
  if(radiusContainsQuadTree(_boid) || (!isEmpty && !isParent))
  {
    for(auto &neighbour : m_allLeaves)
    {
      _boid->think(*neighbour);
    }
  }
  /*else if(!isEmpty && !isParent)
  {
    if(_boid != m_leaf)
    {
      _boid->think(*m_leaf);
    }
  }*/
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

// If there is any overlap at all between boid radius and quadrant.
// The first test fails positive: some non-overlapping quadrants return true.
bool QuadTree::isNearQuadrant(Boid *_boid)
{
  // The diagonal length of the square, and the radius around the boid
  // are used to see if the quadrant is possibly inside the radius.
  // If so, the boids in the quadrant may be neighbours to consider.

  // Assume the container is a Square/Cube
  // for Octant cubes, use sqrt(3)
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
      //ngl::Colour boidColour = m_leaf->getColour();
      //M = m_leaf->getTransformation() * _globalTransformationMatrix;
      ngl::Colour boidColour = m_allLeaves[0]->getColour();
      M = m_allLeaves[0]->getTransformation() * _globalTransformationMatrix;
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
    else
    {
      for(auto &child : m_successors)
      {
        child->draw(_globalTransformationMatrix, _V, _VP);
      }
    }
  }
}
