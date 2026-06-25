#pragma once
#define GL_GLEXT_PROTOTYPES 1
#include <Eigen/Core>
#include <GL/gl.h>
#include <memory>

namespace kd_slam {

  struct Shader {
    unsigned int _shader_id=0;
    GLenum shader_type;
    int source_fd=-1;
    Shader(std::string shader_path, GLenum shader_type_=GL_VERTEX_SHADER);
    ~Shader();
  };
  using ShaderPtr=std::shared_ptr<Shader>;


  
  struct ShaderProgram {
    unsigned int _program_id=0;
    unsigned int _model_pose_location=0;
    unsigned int _object_pose_location=0;
    unsigned int _projection_location=0;
    unsigned int _light_direction_location=0;
    ShaderProgram(ShaderPtr vertex_shader,
                  ShaderPtr fragment_shader);
    virtual ~ShaderProgram();
    ShaderPtr _vertex_shader;
    ShaderPtr _fragment_shader;
  };
  
  using ShaderProgramPtr=std::shared_ptr<ShaderProgram> ;
  
  struct VBOBase {
    ShaderProgramPtr _shader_program;
    VBOBase(ShaderProgramPtr shader_program):
      _shader_program(shader_program){}
    
    void callShader(const Eigen::Matrix4f& projection,
                    const Eigen::Matrix4f& model_pose,
                    const Eigen::Matrix4f& object_pose,
                    const Eigen::Vector3f& light_direction);
    
    virtual void draw(const Eigen::Matrix4f& projection,
                      const Eigen::Matrix4f& model_pose,
                      const Eigen::Matrix4f& object_pose,                      
                      const Eigen::Vector3f& light_direction) = 0;

  };

 
}
