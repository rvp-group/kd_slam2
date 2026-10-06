#pragma once
#include <Eigen/Geometry>
#include <array>
#include <string>
#include <vector>
#include "json_item.h"

struct OusterConfigParams {
  bool phase_lock_enable;
  int sync_pulse_out_frequency;
  int udp_port_imu;
  int udp_port_lidar;
  int nmea_leap_seconds;
  int sync_pulse_out_pulse_width;
  std::string timestamp_mode;
  std::string udp_profile_lidar;
  std::string udp_dest;
  int phase_lock_offset;
  std::string lidar_mode;
  std::string nmea_in_polarity;
  std::string sync_pulse_in_polarity;
  int signal_multiplier;
  int sync_pulse_out_angle;
  std::string udp_profile_imu;
  std::array<int, 2> azimuth_window;
  int columns_per_packet;
  std::string nmea_baud_rate;
  std::string sync_pulse_out_polarity;
  int nmea_ignore_valid_char;
  std::string multipurpose_io_mode;
  std::string operating_mode;
};

struct OusterCalibrationStatus {
  bool reflectivity_valid;
  std::string reflectivity_timestamp;
};

struct OusterLidarIntrinsics {
  Eigen::Isometry3f lidar_to_sensor_transform;
};

struct OusterImuIntrinsics {
  Eigen::Isometry3f imu_to_sensor_transform;
};

struct OusterSensorInfo {
  std::string prod_line;
  int initialization_id;
  std::string build_date;
  std::string status;
  std::string prod_pn;
  std::string image_rev;
  std::string build_rev;
  std::string prod_sn;
};

struct OusterBeamIntrinsics {
  std::vector<float> beam_azimuth_angles;
  float lidar_origin_to_beam_origin_mm;
  Eigen::Isometry3f beam_to_lidar_transform;
  std::vector<float> beam_altitude_angles;
};

struct OusterLidarDataFormat {
  int pixels_per_column;
  std::vector<int> pixel_shift_by_row;
  std::string udp_profile_lidar;
  int columns_per_packet;
  int columns_per_frame;
  std::array<int, 2> column_window;
  std::string udp_profile_imu;
};

struct OusterMetadata {
  OusterConfigParams config_params;
  OusterCalibrationStatus calibration_status;
  OusterLidarIntrinsics lidar_intrinsics;
  OusterImuIntrinsics imu_intrinsics;
  OusterSensorInfo sensor_info;
  OusterBeamIntrinsics beam_intrinsics;
  OusterLidarDataFormat lidar_data_format;
};

// Walks the parsed JSON tree (JsonParser::parse() on a sensor
// metadata/config response) and fills `out`. Throws (via map::at() /
// JsonItemBase's asX()) if a field is missing or not the expected type.
void fillOusterMetadata(JsonMapItem& root, OusterMetadata& out);
