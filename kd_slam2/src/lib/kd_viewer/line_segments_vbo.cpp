#include "line_segments_vbo.h"

namespace kd_slam {
  using namespace std;
  ShaderProgramPtr LineSegmentsVBO::_my_program;

  ShaderProgramPtr LineSegmentsVBO::getShaderProgram() {
    if (!_my_program) {
      ShaderPtr vertex_shader(new Shader("line_segments_vbo.vert", GL_VERTEX_SHADER));
      ShaderPtr fragment_shader(new Shader("line_segments_vbo.frag", GL_FRAGMENT_SHADER));
      _my_program.reset(new ShaderProgram(vertex_shader, fragment_shader));
    }
    return _my_program;
  }

  void LineSegmentsVBO::addSegment(const Eigen::Vector3f& from,
                                   const Eigen::Vector3f& to,
                                   const Eigen::Vector3f& color) {
    std::lock_guard<std::mutex> guard (draw_mutex);
    segments.push_back(from);
    segments.push_back(color);
    segments.push_back(to);
    segments.push_back(color);
    _size=-1;
  }
  void LineSegmentsVBO::bind() {
    if (! isChanged())
      return;
    glGenVertexArrays(1, &_gl_vertex_array);
    glGenBuffers(1, &_gl_vertex_buffer);
    glBindVertexArray(_gl_vertex_array);
    glBindBuffer(GL_ARRAY_BUFFER, _gl_vertex_buffer);
    glBufferData(GL_ARRAY_BUFFER,
                 sizeof(Eigen::Vector3f) * segments.size(),
                 &(segments)[0],
                 GL_STATIC_DRAW);
    _size=segments.size()/2;
  }
  void LineSegmentsVBO::unbind() {
    if (_gl_vertex_array)
      glDeleteVertexArrays(1, &_gl_vertex_array);
    _gl_vertex_array=0;
    if (_gl_vertex_buffer)
      glDeleteBuffers(1, &_gl_vertex_buffer);
    _gl_vertex_buffer=0;
    _size=-1;
  }
  
  void LineSegmentsVBO::clear() {
    std::lock_guard<std::mutex> guard (draw_mutex);
    segments.clear();
    unbind();
  }
  
  LineSegmentsVBO::LineSegmentsVBO()
    :VBOBase(LineSegmentsVBO::getShaderProgram())
  {
  }

  void LineSegmentsVBO::draw(const Eigen::Matrix4f& projection,
                                   const Eigen::Matrix4f& model_pose,
                                   const Eigen::Matrix4f& object_pose,
                                   const Eigen::Vector3f& light_direction) {
    if (! segments.size())
      return;
    bind();
    callShader(projection, model_pose, object_pose, light_direction);
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
                          (const void*) sizeof(Eigen::Vector3f));
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    glDrawArrays(GL_LINES, 0, segments.size());
  }

  LineSegmentsVBO::~LineSegmentsVBO(){
    unbind();
  }
} // namespace kd_slam
