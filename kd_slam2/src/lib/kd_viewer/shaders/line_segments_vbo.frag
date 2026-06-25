#version 330 core
#extension GL_ARB_separate_shader_objects : enable
out vec4 FragColor;
in vec3  my_color;
void main()
{
  FragColor = vec4(my_color, 1); // RGBD
};
