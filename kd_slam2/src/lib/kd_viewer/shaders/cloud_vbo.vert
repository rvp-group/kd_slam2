#version 330 core
#extension GL_ARB_separate_shader_objects : enable
layout (location = 0) in vec3 coords;
layout (location = 1) in vec3 normal_endpoint;
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
  
  mat3 R = mat3(object_pose);
  vec3 n = R*(normal_endpoint-coords)*5.;
  float i_min=0.5;
  float i = dot(n,light_direction);  // RGBD
  if (i<i_min)
    i=i_min;
  my_color = abs(input_color*i);
  //       my_color = n;
}
