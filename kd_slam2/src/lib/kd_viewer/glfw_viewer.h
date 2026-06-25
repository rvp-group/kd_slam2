#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "glad/glad.h"
#include <GLFW/glfw3.h>
#include <functional>
#include <list>
#include "palette.h"

namespace kd_slam {
  struct GLFWViewer {
    GLFWwindow* window=0;
    GLFWViewer();
    virtual ~GLFWViewer();
    virtual void loop();
    std::function<bool(void)> loop_callback;
    virtual bool handleKey(int key, int cloudcode, int action, int mods);
    
    // transformation that maps the CV convention to the gl convention
    static const Eigen::Isometry3f gl_camera_transform; 
    const unsigned int SCR_WIDTH = 640;
    const unsigned int SCR_HEIGHT = 500;
    float fov   =  45.0f;
    enum ManipulationMode {None, Translate, Rotate};
    enum ManipulationOrigin {Camera, World};
  
    ManipulationMode manipulation_mode = None;
    ManipulationOrigin manipulation_origin = Camera;
    bool shift_pressed = false;
    bool alt_pressed = false;
    Eigen::Vector3f light_direction;

    Eigen::Vector2f start_mouse;
    Eigen::Vector2f current_mouse;
    Eigen::Isometry3f camera_pose_start=Eigen::Isometry3f::Identity();
    Eigen::Isometry3f camera_pose=Eigen::Isometry3f::Identity();
    static void key_callback(GLFWwindow* window, int key, int cloudcode, int action, int mods);
    static void framebuffer_size_callback(GLFWwindow* window, int width, int height);
    static void mouse_callback(GLFWwindow* window, double xpos, double ypos);
    static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
    static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

    static Eigen::Matrix4f computeGLPerspectiveMatrix(float fov,
                                                      float width,
                                                      float height,
                                                      float near,
                                                      float far);
#ifdef _PAPER_MODE_
    PaletteWorld _palette_world={
      Eigen::Vector4f(1.,1.,1.,1.),
      Eigen::Vector4f(0.,0.,0.,0.)
    };
#else
    PaletteWorld _palette_world={
      Eigen::Vector4f(.1,.1,.1,1.),
      Eigen::Vector4f(.5,.5,.5,1)
    };
#endif
    
  protected:
    virtual void _drawScene(){}
    virtual const char* _getHUD() const {return 0;}
    virtual const std::list<std::string>& _getLog() const  {return _empty;}
    bool _draw_hud=true;
    bool _draw_log=true;
    int _log_lines=10;
    float _log_width=0.5;
    int _hud_lines=3;
    float _hud_width=0.5;
    void _drawHUD(const char* hud);
    void _drawLog(const std::list<std::string>& log);
    static std::list<std::string> _empty;
  };

}
