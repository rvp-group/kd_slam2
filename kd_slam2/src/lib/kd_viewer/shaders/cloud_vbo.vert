#version 330 core
#extension GL_ARB_separate_shader_objects : enable
layout (location = 0) in vec3 coords;
layout (location = 1) in vec3 normal;
layout (location = 2) in uint packed_color;
uniform mat4 model_pose;
uniform mat4 object_pose;
uniform mat4 projection;
uniform vec3 light_direction;
uniform vec3 input_color;
out vec4 my_color;
void main()
{
  gl_Position =
    projection
    *model_pose
    *object_pose
    *vec4(coords.x, coords.y, coords.z, 1);
  
  mat3 R = mat3(object_pose);
  vec3 n = R*normal;
  float i_min=0.5;
  float i = dot(n,light_direction);  // RGBD
  vec4 point_color;
  if (input_color.x<0.0 || input_color.y<0.0 || input_color.z<0.0) {
    point_color.r=float((packed_color)&0xFFu);
    point_color.g=float((packed_color >> 8u)&0xFFu);
    point_color.b=float((packed_color >> 16u)&0xFFu);
    point_color.a=float((packed_color >> 24u)&0xFFu);
    point_color *= (1./255.);
  } else {
    point_color.r=input_color.x;
    point_color.g=input_color.y;
    point_color.b=input_color.z;
    point_color.a=1.0;
  }
    
  if (i<i_min)
    i=i_min;
  my_color = abs(point_color*i);
  //       my_color = n;
}
