#pragma once
#include <Eigen/Core>

struct PaletteWorld {
  Eigen::Vector4f bg_color;
  Eigen::Vector4f axes_color;
};

struct PaletteFrame {
  Eigen::Vector3f cloud_color;
  Eigen::Vector3f sensor_color;
  Eigen::Vector3f factor_color;
};


  
