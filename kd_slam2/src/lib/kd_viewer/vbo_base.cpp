#include "vbo_base.h"
#include <iostream>
#include <string>
#include <cstdlib>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <cassert>
#include <sys/mman.h>
#include <fstream>



namespace kd_slam {
  using namespace std;
  static const char* PCD_SLAM_VIEWER_SHADERS_PATH=SHADERS_PATH ;
  static constexpr int INFO_LOG_SIZE=512;
  
  Shader::Shader(std::string shader_path, GLenum shader_type_) {
    shader_type=shader_type_;
    std::string vertex_shader_path=std::string(PCD_SLAM_VIEWER_SHADERS_PATH)+"/"+shader_path;
    std::cerr << "loading shader [" << vertex_shader_path << "]" << endl;
    std::vector<std::string> strings;
    ifstream is(vertex_shader_path);
    while(is){
      constexpr int  LINE_LENGTH=1024;
      char buf[LINE_LENGTH];
      is.getline(buf,LINE_LENGTH);
      string s(buf);
      s+=std::string("\n");
      strings.push_back(s);
    }
    std::vector<const char*> lines(strings.size());
    for(size_t i=0; i<strings.size(); ++i)
      lines[i]=&(strings[i][0]);

    _shader_id = glCreateShader(shader_type);
    if (_shader_id==0) {
      throw std::runtime_error("can't create a shader");
    }
    glShaderSource(_shader_id, lines.size(), lines.data(), 0);

    glCompileShader(_shader_id);
    int success;
    char info_log[INFO_LOG_SIZE];
    glGetShaderiv(_shader_id, GL_COMPILE_STATUS, &success);
    if (!success) {
      cerr <<"program: [" <<vertex_shader_path <<"]" << endl;
      for (auto c: lines) {
        cerr << c << endl;
      }
      glGetShaderInfoLog(_shader_id, INFO_LOG_SIZE, NULL, info_log);
      std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << info_log << std::endl;
      throw std::runtime_error("ABORT");
    }
  }
  Shader::~Shader() {
    glDeleteShader(_shader_id);
  }
   
  ShaderProgram::ShaderProgram(ShaderPtr vertex_shader,
                               ShaderPtr fragment_shader):
    _vertex_shader(vertex_shader),
    _fragment_shader(fragment_shader) {
    if (_vertex_shader->shader_type!=GL_VERTEX_SHADER) {
      throw std::runtime_error("ShaderProgram, the vertex shader parameter is not a vertex shader");
    }
    if (_fragment_shader->shader_type!=GL_FRAGMENT_SHADER) {
      throw std::runtime_error("ShaderProgram, the fragment shader parameter is not a fragment shader");
    }
    
    _program_id = glCreateProgram();
    glAttachShader(_program_id, _vertex_shader->_shader_id);
    glAttachShader(_program_id, _fragment_shader->_shader_id);
    glLinkProgram(_program_id);
    int success;
    char info_log[INFO_LOG_SIZE];
    glGetProgramiv(_program_id, GL_LINK_STATUS, &success);
    if (!success) {
      glGetProgramInfoLog(_program_id, INFO_LOG_SIZE, NULL, info_log);
      std::cout << "ERROR::SHADER::PROGRAM::LINK_FAILED\n" << info_log << std::endl;
      throw std::runtime_error("ShaderProgram: cannot link the shaders");
    }
    _projection_location      = glGetUniformLocation(_program_id, "projection");
    _model_pose_location      = glGetUniformLocation(_program_id, "model_pose");
    _object_pose_location     = glGetUniformLocation(_program_id, "object_pose");
    _light_direction_location = glGetUniformLocation(_program_id, "light_direction");
  }

  ShaderProgram::~ShaderProgram() {
    glDeleteProgram(_program_id);
    // cerr << "shader " << this << " dtor" << endl;
  }

  void VBOBase::callShader(const Eigen::Matrix4f& projection,
                           const Eigen::Matrix4f& model_pose,
                           const Eigen::Matrix4f& object_pose,
                           const Eigen::Vector3f& light_direction) {
    auto ld = light_direction.normalized();
    glUseProgram(_shader_program->_program_id);
    glUniformMatrix4fv(_shader_program->_projection_location, 1, GL_FALSE, projection.data());
    glUniformMatrix4fv(_shader_program->_model_pose_location, 1, GL_FALSE, model_pose.data());
    glUniformMatrix4fv(_shader_program->_object_pose_location, 1, GL_FALSE, object_pose.data());
    glUniform3fv(_shader_program->_light_direction_location, 1, ld.data());
  }

} // namespace kd_slam
