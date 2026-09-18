#pragma once
#include "vbo_base.h"
#include <iostream>

namespace kd_slam {
  struct PointNormal3fKDCloudVBO: public VBOBase {

    struct PointType {
      Eigen::Vector3f point;
      Eigen::Vector3f normal;
      uint8_t rgba[4];
    };
      
    PointNormal3fKDCloudVBO(const std::vector<PointType>& cloud_);
    void updateBuffer(const std::vector<PointType>& cloud_);
    ~PointNormal3fKDCloudVBO();
    void draw(const Eigen::Matrix4f& projection,
              const Eigen::Matrix4f& model_pose,
              const Eigen::Matrix4f& object_pose,
              const Eigen::Vector3f& light_direction) override;
    inline size_t size() const {return _size;}
    inline size_t capacity() const {return _capacity;}
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
      // builds its own dedicated (point, point+normal*scale) line-segment buffer
      void init(const std::vector<PointType>& cloud_, float normal_scale=0.2f);
      int decimation = 1;
   protected:
      static ShaderProgramPtr _my_program;
      unsigned int _gl_vertex_buffer = 0, _gl_vertex_array = 0;
      unsigned int _color_location=0;
      size_t _size=0;
      size_t _capacity=0;
   };

    Eigen::Vector3f point_color;
    int decimation = 1;
    bool show_normals = false;
  protected:
    unsigned int _gl_vertex_buffer = 0, _gl_vertex_array = 0;
    size_t _size=0;
    size_t _capacity=0;
    static ShaderProgramPtr _my_program;
    unsigned int _color_location=0;
    NormalsVBO _normals_vbo;
  };
} 
