#ifndef Scene_H__
#define Scene_H__
#include <ngl/Camera.h>
#include <ngl/Colour.h>
#include <ngl/Light.h>
#include <ngl/Transformation.h>
#include <ngl/Text.h>
#include <ngl/Obj.h>
#include <memory>
#include <QOpenGLWindow>

#include "FlockFactory.h"
#include "Flock.h"

//----------------------------------------------------------------------------------------------------------------------
/// @file Scene.h
/// @brief this class inherits from the Qt OpenGLWindow and allows us to use NGL to draw OpenGL
/// @author Jonathan Macey
/// @version 1.0
/// @date 10/9/13
/// Revision History :
/// This is an initial version used for the new NGL6 / Qt 5 demos
/// @class Scene
/// @brief our main glwindow widget for NGL applications all drawing elements are
/// put in this file
//----------------------------------------------------------------------------------------------------------------------

class Scene : public QOpenGLWindow
{
  public:
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief ctor for our NGL drawing class
    /// @param [in] parent the parent window to the class
    //----------------------------------------------------------------------------------------------------------------------
    Scene();
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief dtor must close down ngl and release OpenGL resources
    //----------------------------------------------------------------------------------------------------------------------
    ~Scene();
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief the initialize class is called once when the window is created and we have a valid GL context
    /// use this to setup any default GL stuff
    //----------------------------------------------------------------------------------------------------------------------
    void initializeGL();
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief this is called everytime we want to draw the scene
    //----------------------------------------------------------------------------------------------------------------------
    void paintGL();
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief this is called everytime we resize
    //----------------------------------------------------------------------------------------------------------------------
    // Qt 5.5.1 must have this implemented and uses it
    void resizeGL(QResizeEvent *_event);
    // Qt 5.x uses this instead! http://doc.qt.io/qt-5/qopenglwindow.html#resizeGL
    void resizeGL(int _w, int _h);
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief  toggle the animation and simulation
    //----------------------------------------------------------------------------------------------------------------------
    inline void toggleAnimation(){m_animate ^=true;}
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief toggle drawing wireframe meshes
    //----------------------------------------------------------------------------------------------------------------------
    inline void toggleWireframe(){m_wireframe^=true;}

  private:
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief used to store the x/y rotation mouse value
    //----------------------------------------------------------------------------------------------------------------------
    int m_spinXFace;
    int m_spinYFace;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief the previous x/y mouse value for translation and rotation
    //----------------------------------------------------------------------------------------------------------------------
    int m_origXPos;
    int m_origYPos;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief window Width/Height dimensions
    //----------------------------------------------------------------------------------------------------------------------
    int m_width = 1024;
    int m_height = 720;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief Our Camera
    //----------------------------------------------------------------------------------------------------------------------
    ngl::Camera m_cam;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief body transform matrix
    //----------------------------------------------------------------------------------------------------------------------
    ngl::Mat4 m_bodyTransform;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief world / mouse transform
    //----------------------------------------------------------------------------------------------------------------------
    ngl::Mat4 m_globalTransformMatrix;
    //----------------------------------------------------------------------------------------------------------------------
    /// @brief the model position for mouse movement
    //----------------------------------------------------------------------------------------------------------------------
    ngl::Vec3 m_modelPos;

    /// @brief Assortment of flock parameters
    const int m_flockSize = 8000;
    ngl::Vec3 m_flockOrigin;
    const float m_velClamp = 0.6f;
    const float m_turnClamp = 0.95f;
    const float m_avoidRadius = 16.0f;
    const float m_approachRadius = 32.0f;
    const float m_fieldOfView = -0.25f;
    const int m_neighbourLimit = 20;

    const FlockType m_flockType = FlockType::BINARY;
    std::unique_ptr<Flock> m_flock;

    /// @brief To help debug behaviour of a specific Boid.
    int m_inspectIndex;

    /// @brief flag to animate / simulate
    bool m_animate;

    /// @brief flag for wireframe drawing
    bool m_wireframe;

    ///@brief text for rendering
    std::unique_ptr<ngl::Text>m_text;

    /// @brief method to load transform matrices to the shader
    void loadMatricesToShader();

    /// @brief Qt Event called for keyboard interaction
    /// @param [in] _event the Qt event to query for size etc
    void keyPressEvent(QKeyEvent *_event);

    /// @brief Qt Event for mouse interaction
    /// inherited from QObject and overridden here.
    /// @param _event the Qt Event structure
    void mouseMoveEvent (QMouseEvent *_event );
    void mousePressEvent (QMouseEvent *_event);
    void wheelEvent(QWheelEvent *_event);

    /// @brief timer event called when timer triggers
    void timerEvent(QTimerEvent *);
};

#endif
