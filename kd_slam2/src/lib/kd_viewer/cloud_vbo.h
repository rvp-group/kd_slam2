#pragma once
#include "vbo_base.h"
#include <iostream>

namespace kd_slam {
  struct PointNormal3fKDCloudVBO: public VBOBase {
    PointNormal3fKDCloudVBO(const std::vector<Eigen::Vector3f>& cloud_);
    ~PointNormal3fKDCloudVBO();
    void draw(const Eigen::Matrix4f& projection,
              const Eigen::Matrix4f& model_pose,
              const Eigen::Matrix4f& object_pose,
              const Eigen::Vector3f& light_direction) override;
    inline size_t size() const {return _size;}
    static ShaderProgramPtr getShaderProgram();

    struct NormalsVBO: public VBOBase {
      NormalsVBO();
      ~NormalsVBO();
      void draw(const Eigen::Matrix4f& projection,
                const Eigen::Matrix4f& model_pose,
                const Eigen::Matrix4f& object_pose,
                const Eigen::Vector3f& light_direction) override;
      Eigen::Vector3f normal_color;
      static ShaderProgramPtr getShaderProgram();
      void init(unsigned int gl_buf, size_t s); 
      int decimation = 1;
   protected:
      static ShaderProgramPtr _my_program;
      unsigned int _gl_vertex_buffer = 0, _gl_vertex_array = 0;
      unsigned int _color_location=0;
      size_t _size; 
   };

    Eigen::Vector3f point_color;
    int decimation = 1;
  protected:
    unsigned int _gl_vertex_buffer = 0, _gl_vertex_array = 0;
    size_t _size=0;
    static ShaderProgramPtr _my_program;
    unsigned int _color_location=0;
    NormalsVBO _normals_vbo;
  };
} 
