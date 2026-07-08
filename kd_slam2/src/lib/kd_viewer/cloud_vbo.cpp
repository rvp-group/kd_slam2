#include "cloud_vbo.h"
#include <iostream>
#include <algorithm>

namespace kd_slam {
  using namespace std;
  
  ShaderProgramPtr PointNormal3fKDCloudVBO::_my_program;
  ShaderProgramPtr PointNormal3fKDCloudVBO::NormalsVBO::_my_program;

  ShaderProgramPtr PointNormal3fKDCloudVBO::getShaderProgram() {
    if (!_my_program) {
      ShaderPtr vertex_shader(new Shader("cloud_vbo.vert", GL_VERTEX_SHADER));
      ShaderPtr fragment_shader(new Shader("cloud_vbo.frag", GL_FRAGMENT_SHADER));
      _my_program.reset(new ShaderProgram(vertex_shader, fragment_shader));
      if (vertex_shader->_shader_id==0)
        throw std::runtime_error("no vertex shader loaded");
      if (fragment_shader->_shader_id==0)
        throw std::runtime_error("no fragment shader loaded");
      if (_my_program->_program_id == 0)
        throw std::runtime_error("no GL program compiled");
    }
    return _my_program;
  }

  ShaderProgramPtr PointNormal3fKDCloudVBO::NormalsVBO::getShaderProgram() {
    if (!_my_program) {
      ShaderPtr vertex_shader(new Shader("cloud_vbo_normal.vert", GL_VERTEX_SHADER));
      ShaderPtr fragment_shader(new Shader("line_segments_vbo.frag", GL_FRAGMENT_SHADER));
      _my_program.reset(new ShaderProgram(vertex_shader, fragment_shader));
      if (vertex_shader->_shader_id==0)
        throw std::runtime_error("no vertex shader loaded");
      if (fragment_shader->_shader_id==0)
        throw std::runtime_error("no fragment shader loaded");
      if (_my_program->_program_id == 0)
        throw std::runtime_error("no GL program compiled");
    }
    return _my_program;
  }
  
  PointNormal3fKDCloudVBO::NormalsVBO::NormalsVBO()
    :VBOBase(PointNormal3fKDCloudVBO::NormalsVBO::getShaderProgram()) {
    _color_location = glGetUniformLocation(_shader_program->_program_id, "input_color");
  }
  
  PointNormal3fKDCloudVBO::PointNormal3fKDCloudVBO(const std::vector<Eigen::Vector3f>& cloud_)
    :VBOBase(PointNormal3fKDCloudVBO::getShaderProgram())
  {
    
    if (cloud_.size()%2) {
      throw std::runtime_error("odd number of points in cloud");
    }
    _color_location = glGetUniformLocation(_shader_program->_program_id, "input_color");

    std::vector cloud(cloud_);
    for (size_t i=1; i<cloud.size(); i+=2) {
      cloud[i]=cloud[i]*0.2+cloud[i-1];
    }
    point_color=Eigen::Vector3f(0.5, 0.5, 0.5);
    // copy data on GPU
    glGenVertexArrays(1, &_gl_vertex_array);
    glBindVertexArray(_gl_vertex_array);
    glGenBuffers(1, &_gl_vertex_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, _gl_vertex_buffer);
    glBufferData(GL_ARRAY_BUFFER,
                 sizeof(Eigen::Vector3f) * cloud.size(),
                 &(cloud)[0],
                 GL_STATIC_DRAW);

    // specify data layout
    glBindVertexArray(_gl_vertex_array);
    glVertexAttribPointer(0,
                          3,
                          GL_FLOAT,
                          GL_FALSE,
                          2*sizeof(Eigen::Vector3f),
                          (const void*)0);
    
    glVertexAttribPointer(1,
                          3,
                          GL_FLOAT,
                          GL_FALSE,
                          2*sizeof(Eigen::Vector3f),
                          (const void*)sizeof(Eigen::Vector3f));
    _size=cloud.size()/2;
    _normals_vbo.init(_gl_vertex_buffer, cloud.size());

  }


  void PointNormal3fKDCloudVBO::draw(const Eigen::Matrix4f& projection,
                                   const Eigen::Matrix4f& model_pose,
                                   const Eigen::Matrix4f& object_pose,
                                   const Eigen::Vector3f& light_direction) {
    callShader(projection, model_pose, object_pose, light_direction);
    glUniform3fv(_color_location, 1, point_color.data());
    glBindVertexArray(_gl_vertex_array);
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    glDrawArrays(GL_POINTS, 0, _size / std::max(1, decimation));
    /*
    _normals_vbo.normal_color=point_color;
    _normals_vbo.decimation=decimation;
    
    _normals_vbo.draw(projection,
                      model_pose,
                      object_pose,
                      light_direction);
    */
  }

  PointNormal3fKDCloudVBO::~PointNormal3fKDCloudVBO(){
    glDeleteVertexArrays(1, &_gl_vertex_array);
    glDeleteBuffers(1, &_gl_vertex_buffer);
  }


  void PointNormal3fKDCloudVBO::NormalsVBO::init(unsigned int gl_vbuf, size_t s) {
    _size=s;
    _gl_vertex_buffer=gl_vbuf;
    // specify data layout
    glGenVertexArrays(1, &_gl_vertex_array);
    glBindVertexArray(_gl_vertex_array);
    glBindBuffer(GL_ARRAY_BUFFER, _gl_vertex_buffer);
    glVertexAttribPointer(0,
                          3,
                          GL_FLOAT,
                          GL_FALSE,
                          sizeof(Eigen::Vector3f),
                          (const void*)0);
  }

  void PointNormal3fKDCloudVBO::NormalsVBO::draw(const Eigen::Matrix4f& projection,
                                     const Eigen::Matrix4f& model_pose,
                                     const Eigen::Matrix4f& object_pose,
                                     const Eigen::Vector3f& light_direction) {
    glLineWidth(1);
    callShader(projection, model_pose, object_pose, light_direction);
    glUniform3fv(_color_location, 1, normal_color.data());
    glBindVertexArray(_gl_vertex_array);
    glEnableVertexAttribArray(0);
    glDrawArrays(GL_LINES, 0, (_size / std::max(1, decimation)) & ~1);
  }

  PointNormal3fKDCloudVBO::NormalsVBO::~NormalsVBO(){
    glDeleteVertexArrays(1, &_gl_vertex_array);
  }

} // namespace kd_slam
