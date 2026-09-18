#version 330 core
#extension GL_ARB_separate_shader_objects : enable
out vec4 FragColor;
in vec4  my_color;
void main()
{
  FragColor = my_color; // RGBD
};
