#version 330 core
#extension GL_ARB_separate_shader_objects : enable
layout (location = 0) in vec3 coords;
layout (location = 1) in vec3 color;
uniform mat4 model_pose;
uniform mat4 object_pose;
uniform mat4 projection;
uniform vec3 light_direction;
out vec3 my_color;
void main()
{
  gl_Position =
    projection
    *model_pose
    *object_pose
    *vec4(coords.x, coords.y, coords.z, 1);
  my_color = color;
}
