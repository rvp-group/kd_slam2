#pragma once
#include "vbo_base.h"
#include <iostream>
#include <mutex>

namespace kd_slam {
  struct LineSegmentsVBO: public VBOBase{
    std::mutex draw_mutex;
    
    LineSegmentsVBO();
    ~LineSegmentsVBO();
    void addSegment(const Eigen::Vector3f& from,
                    const Eigen::Vector3f& to,
                    const Eigen::Vector3f& color =Eigen::Vector3f(1,0,0));
    void clear();
    
    void draw(const Eigen::Matrix4f& projection,
              const Eigen::Matrix4f& model_pose,
              const Eigen::Matrix4f& object_pose,
              const Eigen::Vector3f& light_direction) override;
    inline size_t size() const {return _size;}
    static ShaderProgramPtr getShaderProgram();
    std::vector<Eigen::Vector3f> segments; //p_from, c_from, p_to, c_to
  protected:
    inline bool isChanged() const  {return _size!= (int) (segments.size()/2);}
    void bind();
    void unbind();
    bool is_changed=false;
    int _size=0;
    static ShaderProgramPtr _my_program;
    unsigned int
    _gl_vertex_buffer = 0,
      _gl_vertex_array = 0;
  };
} // namespace kd_slam
