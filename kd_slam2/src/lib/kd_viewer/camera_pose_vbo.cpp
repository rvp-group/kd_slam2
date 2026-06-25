#include "camera_pose_vbo.h"
#include <unistd.h>

#include <iostream>
namespace kd_slam {
  using namespace std;

  ShaderProgramPtr CameraPoseVBO::getShaderProgram() {
    if (!_my_program) {
      ShaderPtr vertex_shader(new Shader("camera_pose_vbo.vert", GL_VERTEX_SHADER));
      ShaderPtr fragment_shader(new Shader("camera_pose_vbo.frag", GL_FRAGMENT_SHADER));
      _my_program.reset(new ShaderProgram(vertex_shader, fragment_shader));
    }
    return _my_program;
  }

  CameraPoseVBO::CameraPoseVBO(float width, float height) : VBOBase(getShaderProgram()) {
    // cerr << "num_coordinates: " << num_coordinates << endl;
    // cerr << "num_vertices: " << num_vertices << endl;

    for (int i = 0; i < num_coordinates; i += 3) {
      _vertices[i]     = _unscaled_vertices[i] * width / 2;
      _vertices[i + 1] = _unscaled_vertices[i + 1] * width / 2;
      _vertices[i + 2] = _unscaled_vertices[i + 2] * height;
    }
    
    glGenVertexArrays(1, &_gl_vertex_array);
    glGenBuffers(1, &_gl_vertex_buffer);
    glBindVertexArray(_gl_vertex_array);
    glBindBuffer(GL_ARRAY_BUFFER, _gl_vertex_buffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(_vertices), _vertices, GL_STATIC_DRAW);
    glBindVertexArray(_gl_vertex_array);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), 0);
  }
  CameraPoseVBO::~CameraPoseVBO() {
    // cerr << __PRETTY_FUNCTION__ << this << " dtor" << endl;
    glDeleteVertexArrays(1, &_gl_vertex_array);
    glDeleteBuffers(1, &_gl_vertex_buffer);
  }

  void CameraPoseVBO::draw(const Eigen::Matrix4f& projection,
                           const Eigen::Matrix4f& model_pose,
                           const Eigen::Matrix4f& object_pose,
                           const Eigen::Vector3f& light_direction) {

    callShader(projection, model_pose, object_pose, light_direction);
    glBindVertexArray(_gl_vertex_array);
    glEnableVertexAttribArray(0);
    glLineWidth(3);
    glDrawArrays(GL_LINE_STRIP, 0, num_vertices);
  }

  ShaderProgramPtr CameraPoseVBO::_my_program;

  float CameraPoseVBO::_vertices[CameraPoseVBO::num_coordinates];

} // namespace kd_slam
