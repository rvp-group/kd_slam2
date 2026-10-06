#include "ouster_metadata.h"
#include <stdexcept>
using namespace std;

namespace {

JsonItemBase& get(JsonMapItem& m, const string& key) {
  auto it = m.find(key);
  if (it == m.end())
    throw std::runtime_error("missing JSON key: " + key);
  return *it->second;
}

JsonItemBase* tryGet(JsonMapItem& m, const string& key) {
  auto it = m.find(key);
  return it == m.end() ? nullptr : it->second.get();
}

Eigen::Isometry3f asIsometry(JsonItemBase& item) {
  auto* vec = item.asVector();
  Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
  for (size_t i = 0; i < 16 && i < vec->size(); ++i)
    m(i / 4, i % 4) = static_cast<float>((*vec)[i]->asDouble());
  Eigen::Isometry3f t;
  t.matrix() = m;
  return t;
}

array<int, 2> asIntPair(JsonItemBase& item) {
  array<int, 2> out{};
  auto* vec = item.asVector();
  for (size_t i = 0; i < 2 && i < vec->size(); ++i)
    out[i] = (*vec)[i]->asInt();
  return out;
}

vector<float> asFloatVector(JsonItemBase& item) {
  auto* vec = item.asVector();
  vector<float> out;
  out.reserve(vec->size());
  for (auto& v : *vec)
    out.push_back(static_cast<float>(v->asDouble()));
  return out;
}

vector<int> asIntVector(JsonItemBase& item) {
  auto* vec = item.asVector();
  vector<int> out;
  out.reserve(vec->size());
  for (auto& v : *vec)
    out.push_back(v->asInt());
  return out;
}

void fillConfigParams(JsonMapItem& m, OusterConfigParams& out) {
  out.phase_lock_enable = get(m, "phase_lock_enable").asInt() != 0;
  out.sync_pulse_out_frequency = get(m, "sync_pulse_out_frequency").asInt();
  out.udp_port_imu = get(m, "udp_port_imu").asInt();
  out.udp_port_lidar = get(m, "udp_port_lidar").asInt();
  out.nmea_leap_seconds = get(m, "nmea_leap_seconds").asInt();
  out.sync_pulse_out_pulse_width = get(m, "sync_pulse_out_pulse_width").asInt();
  out.timestamp_mode = get(m, "timestamp_mode").asString();
  out.udp_profile_lidar = get(m, "udp_profile_lidar").asString();
  out.udp_dest = get(m, "udp_dest").asString();
  out.phase_lock_offset = get(m, "phase_lock_offset").asInt();
  out.lidar_mode = get(m, "lidar_mode").asString();
  out.nmea_in_polarity = get(m, "nmea_in_polarity").asString();
  out.sync_pulse_in_polarity = get(m, "sync_pulse_in_polarity").asString();
  out.signal_multiplier = get(m, "signal_multiplier").asInt();
  out.sync_pulse_out_angle = get(m, "sync_pulse_out_angle").asInt();
  out.udp_profile_imu = get(m, "udp_profile_imu").asString();
  out.azimuth_window = asIntPair(get(m, "azimuth_window"));
  out.columns_per_packet = get(m, "columns_per_packet").asInt();
  out.nmea_baud_rate = get(m, "nmea_baud_rate").asString();
  out.sync_pulse_out_polarity = get(m, "sync_pulse_out_polarity").asString();
  out.nmea_ignore_valid_char = get(m, "nmea_ignore_valid_char").asInt();
  out.multipurpose_io_mode = get(m, "multipurpose_io_mode").asString();
  out.operating_mode = get(m, "operating_mode").asString();
}

void fillCalibrationStatus(JsonMapItem& m, OusterCalibrationStatus& out) {
  auto& reflectivity = *get(m, "reflectivity").asMap();
  out.reflectivity_valid = get(reflectivity, "valid").asInt() != 0;
  out.reflectivity_timestamp = get(reflectivity, "timestamp").asString();
}

void fillLidarIntrinsics(JsonMapItem& m, OusterLidarIntrinsics& out) {
  out.lidar_to_sensor_transform = asIsometry(get(m, "lidar_to_sensor_transform"));
}

void fillImuIntrinsics(JsonMapItem& m, OusterImuIntrinsics& out) {
  out.imu_to_sensor_transform = asIsometry(get(m, "imu_to_sensor_transform"));
}

void fillSensorInfo(JsonMapItem& m, OusterSensorInfo& out) {
  out.prod_line = get(m, "prod_line").asString();
  out.initialization_id = get(m, "initialization_id").asInt();
  out.build_date = get(m, "build_date").asString();
  out.status = get(m, "status").asString();
  out.prod_pn = get(m, "prod_pn").asString();
  out.image_rev = get(m, "image_rev").asString();
  out.build_rev = get(m, "build_rev").asString();
  out.prod_sn = get(m, "prod_sn").asString();
}

void fillBeamIntrinsics(JsonMapItem& m, OusterBeamIntrinsics& out) {
  out.beam_azimuth_angles = asFloatVector(get(m, "beam_azimuth_angles"));
  out.lidar_origin_to_beam_origin_mm =
      static_cast<float>(get(m, "lidar_origin_to_beam_origin_mm").asDouble());
  if (auto* item = tryGet(m, "beam_to_lidar_transform")) {
    out.beam_to_lidar_transform = asIsometry(*item);
  } else {
    // Not always emitted -- it's fully determined by
    // lidar_origin_to_beam_origin_mm: identity rotation, translated by that
    // amount along x. This is Ouster's fixed convention, not per-sensor data.
    out.beam_to_lidar_transform = Eigen::Isometry3f::Identity();
    out.beam_to_lidar_transform.translation().x() = out.lidar_origin_to_beam_origin_mm;
  }
  out.beam_altitude_angles = asFloatVector(get(m, "beam_altitude_angles"));
}

void fillLidarDataFormat(JsonMapItem& m, OusterLidarDataFormat& out) {
  out.pixels_per_column = get(m, "pixels_per_column").asInt();
  out.pixel_shift_by_row = asIntVector(get(m, "pixel_shift_by_row"));
  out.udp_profile_lidar = get(m, "udp_profile_lidar").asString();
  out.columns_per_packet = get(m, "columns_per_packet").asInt();
  out.columns_per_frame = get(m, "columns_per_frame").asInt();
  out.column_window = asIntPair(get(m, "column_window"));
  out.udp_profile_imu = get(m, "udp_profile_imu").asString();
}

}  // namespace

void fillOusterMetadata(JsonMapItem& root, OusterMetadata& out) {
  fillConfigParams(*get(root, "config_params").asMap(), out.config_params);
  fillCalibrationStatus(*get(root, "calibration_status").asMap(), out.calibration_status);
  fillLidarIntrinsics(*get(root, "lidar_intrinsics").asMap(), out.lidar_intrinsics);
  fillImuIntrinsics(*get(root, "imu_intrinsics").asMap(), out.imu_intrinsics);
  fillSensorInfo(*get(root, "sensor_info").asMap(), out.sensor_info);
  fillBeamIntrinsics(*get(root, "beam_intrinsics").asMap(), out.beam_intrinsics);
  fillLidarDataFormat(*get(root, "lidar_data_format").asMap(), out.lidar_data_format);
}
