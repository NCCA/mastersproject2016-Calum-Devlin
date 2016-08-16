#include <QMouseEvent>
#include <QGuiApplication>

#include "Scene.h"
#include "FlockFactory.h"

#include <ngl/Camera.h>
#include <ngl/Light.h>
#include <ngl/Transformation.h>
#include <ngl/Material.h>
#include <ngl/NGLInit.h>
#include <ngl/VAOPrimitives.h>
#include <ngl/ShaderLib.h>
#include <ngl/Random.h>

Scene::Scene()
{
  // mouse rotation values set to 0
  m_spinXFace=0;
  m_spinYFace=0;
  setTitle("Delaunay Boid Demonstration");
  m_animate=false;
  m_randomPlace=false;
  m_bboxDraw=false;
  m_wireframe=false;
  ngl::Random *rng=ngl::Random::instance();
  rng->setSeed();
}

Scene::~Scene()
{
  std::cout<<"Shutting down NGL, removing VAO's and Shaders\n";
}

void Scene::resizeGL(QResizeEvent *_event)
{
  m_width=_event->size().width()*devicePixelRatio();
  m_height=_event->size().height()*devicePixelRatio();
  // now set the camera size values as the screen size has changed
  m_cam.setShape(45.0f,(float)width()/height(),0.05f,350.0f);
}

void Scene::resizeGL(int _w , int _h)
{
  m_cam.setShape(45.0f,(float)_w/_h,0.05f,350.0f);
  m_width=_w*devicePixelRatio();
  m_height=_h*devicePixelRatio();
}

void Scene::initializeGL()
{
  // we must call this first before any other GL commands to load and link the
  // gl commands from the lib, if this is not done program will crash
  ngl::NGLInit::instance();

  // enable depth testing for drawing
  glEnable(GL_DEPTH_TEST);
  // enable multisampling for smoother drawing
  glEnable(GL_MULTISAMPLE);
  glClearColor(0.6f, 0.6f, 0.6f, 1.0f);			   // Grey Background
  // enable depth testing for drawing
  glEnable(GL_DEPTH_TEST);
  glViewport(0,0,width(),height());

  // now to load the shader and set the values
  // grab an instance of shader manager
  ngl::ShaderLib *shader=ngl::ShaderLib::instance();
  (*shader)["nglToonShader"]->use();
  shader->setShaderParam4f("Colour",1,1,1,1);

  (*shader)["nglDiffuseShader"]->use();
  shader->setShaderParam4f("Colour",1,1,0,1);
  shader->setShaderParam3f("lightPos",0,0,1);
  shader->setShaderParam4f("lightDiffuse",1,1,1,1);

  // Now we will create a basic Camera from the graphics library
  // This is a static camera so it only needs to be set once
  // First create Values for the camera position
	ngl::Vec3 from(0,20,200);
  ngl::Vec3 to(0,0,0);
  ngl::Vec3 up(0,1,0);
  // now load to our new camera
  m_cam.set(from,to,up);
  // set the shape using FOV 45 Aspect Ratio based on Width and Height
  // The final two are near and far clipping planes of 0.5 and 10
  m_cam.setShape(50,(float)720.0/576.0,0.05,350);
  ngl::VAOPrimitives *prim = ngl::VAOPrimitives::instance();
  prim->createCone("boid",1,1,4,1);
  prim->createSphere("target",5,2);
  prim->createLineGrid("plane",500,500,500);
  prim->createLineGrid("quadSquare",1,1,1);

  // Prepping for drawing lines.  See Lindenmayer.
  /*ngl::VertexArrayObject myv = ngl::VertexArrayObject::createVOA(GL_LINES);
  myv.bind();

  myv.unbind();*/

  //m_flockBasic.reset(new BasicFlock(450,1.2f));
  std::unique_ptr<FlockFactory> flockMaker(new FlockFactory);
  //m_flock.reset(flockMaker->GenerateFlock(FlockType::NAIVE,&m_flockSize,1.2f, ngl::Vec3(0,0,0)));
  //m_flock.reset(flockMaker->GenerateFlock(FlockType::BINARY,&m_flockSize,1.2f, ngl::Vec3(0,0,0)));
  m_flock.reset(flockMaker->GenerateFlock(FlockType::DELAUNAY,&m_flockSize,1.2f, ngl::Vec3(0,0,0)));
  m_flock->setCam(&m_cam);
  //m_flockBasic->setCam(&m_cam);

  startTimer(10);

  m_text.reset(new  ngl::Text(QFont("Arial",18)));
  m_text->setScreenSize(this->size().width(),this->size().height());
}

void Scene::loadMatricesToShader()
{
  ngl::ShaderLib *shader=ngl::ShaderLib::instance();

  ngl::Mat4 M = m_bodyTransform*m_globalTransformMatrix;
  ngl::Mat4 MV = M*m_cam.getViewMatrix();
  ngl::Mat4 MVP = MV*m_cam.getVPMatrix();
  ngl::Mat3 normalMatrix = MV;
  normalMatrix.inverse();

  shader->setRegisteredUniform("M",M);
  shader->setRegisteredUniform("MV",MV);
  shader->setRegisteredUniform("MVP",MVP);
  shader->setRegisteredUniform("normalMatrix",normalMatrix);
}

void Scene::paintGL()
{
  // clear the screen and depth buffer
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glViewport(0,0,m_width,m_height);
  // grab an instance of the shader manager
  ngl::ShaderLib *shader=ngl::ShaderLib::instance();
  (*shader)["nglColourShader"]->use();

  // Rotation based on the mouse position for our global transform
  ngl::Mat4 rotX;
  ngl::Mat4 rotY;
  // create the rotation matrices
  rotX.rotateX(m_spinXFace);
  rotY.rotateY(m_spinYFace);
  // multiply the rotations
  m_globalTransformMatrix=rotY*rotX;
  // add the translations
  m_globalTransformMatrix.m_m[3][0] = m_modelPos.m_x;
  m_globalTransformMatrix.m_m[3][1] = m_modelPos.m_y;
  m_globalTransformMatrix.m_m[3][2] = m_modelPos.m_z;
  // set this in the TX stack

  if(m_animate)
  {
    m_flock->think();
  }
  m_flock->draw(m_globalTransformMatrix);
  //m_flockBasic->draw(m_globalTransformMatrix);

  shader->setShaderParam4f("Colour",0.0f,0.0f,0.0f,1.0f);

  m_bodyTransform.identity();
  loadMatricesToShader();

  //ngl::VAOPrimitives *prim=ngl::VAOPrimitives::instance();
  //prim->draw("plane");
  m_text->setColour(1,1,1);
  QString text=QString("Number of Boids=%2").arg(m_flockSize);
  m_text->renderText(10,18,text );
}

//----------------------------------------------------------------------------------------------------------------------
/// @brief the increment for x/y/z translation with mouse movement
//----------------------------------------------------------------------------------------------------------------------
static float INCREMENT=0.05;

void Scene::mouseMoveEvent (QMouseEvent * _event)
{
  int diffX = _event->x() - m_origXPos;
  int diffY = _event->y() - m_origYPos;

  // Rotation
  if(_event->buttons() == Qt::LeftButton)
  {
    m_spinXFace += (float) 0.5f * diffY;
    m_spinYFace += (float) 0.5f * diffX;
    m_origXPos = _event->x();
    m_origYPos = _event->y();
  }
  // Translation (X,Y)
  if(_event->buttons() == Qt::RightButton)
  {
    m_origXPos=_event->x();
    m_origYPos=_event->y();
    m_modelPos.m_x += INCREMENT * diffX;
    m_modelPos.m_y -= INCREMENT * diffY;
  }
  // Translation (Z)
  if(_event->buttons() == Qt::MiddleButton)
  {
    m_origYPos = _event->y();
    m_modelPos.m_z += INCREMENT  * diffY;
  }
  update();
}

//----------------------------------------------------------------------------------------------------------------------
void Scene::mousePressEvent ( QMouseEvent * _event)
{
  // This method is called when any mouse button is pressed.
  // Store the value where the mouse was clicked (x,y)
  m_origXPos = _event->x();
  m_origYPos = _event->y();
}

void Scene::wheelEvent(QWheelEvent *_event)
{
  if(_event->delta()>0)
  {
    INCREMENT *=2;
  }
  if(_event->delta()<0)
  {
    INCREMENT *=0.5;
  }
}

//----------------------------------------------------------------------------------------------------------------------
void Scene::keyPressEvent(QKeyEvent *_event)
{
  // this method is called every time the main window recives a key event.
  // we then switch on the key value and set the camera in the Scene
  switch (_event->key())
  {
  // escape key to quit
  case Qt::Key_Escape : QGuiApplication::exit(EXIT_SUCCESS); break;
  // turn on wireframe rendering
  case Qt::Key_W : glPolygonMode(GL_FRONT_AND_BACK,GL_LINE); break;
  // turn off wireframe (solid)
  case Qt::Key_S : glPolygonMode(GL_FRONT_AND_BACK,GL_FILL); break;
  // show full screen
  case Qt::Key_F : showFullScreen(); break;
  // show windowed
  case Qt::Key_N : showNormal(); break;
  case Qt::Key_Space : toggleAnimation(); break;
  case Qt::Key_X : stepAnimation(); break;

  case Qt::Key_B : toggleBBox(); break;
  case Qt::Key_R : toggleRandomPlace(); break;
  case Qt::Key_0 : resetSim(); break;

  default : break;
  }
  // finally update the GLWindow and re-draw
  update();
}

void Scene::resetSim()
{
}

void Scene::timerEvent(QTimerEvent *_e)
{
  //m_flock->update();
  update();
}

void Scene::stepAnimation()
{
}

