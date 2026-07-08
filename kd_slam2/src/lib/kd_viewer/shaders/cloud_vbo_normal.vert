#version 330 core
#extension GL_ARB_separate_shader_objects : enable
layout (location = 0) in vec3 coords;
uniform mat4 model_pose;
uniform mat4 object_pose;
uniform mat4 projection;
uniform vec3 light_direction;
uniform vec3 input_color;
out vec3 my_color;
void main()
{
  gl_Position =
    projection
    *model_pose
    *object_pose
    *vec4(coords.x, coords.y, coords.z, 1);
  my_color=input_color;
}
