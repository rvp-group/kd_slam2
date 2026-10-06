#pragma once
#include <cstdint>
#include <iostream>


// LEGACY lidar UDP packet layout, per
// https://docs.ouster.com/sensor-docs/firmware/3.2/lidar-data-legacy
//
// A packet is columns_per_packet LegacyMeasurementBlockHeader+channels+status
// groups back to back. pixels_per_column (channel count) is a runtime
// value from the sensor's metadata, not fixed here -- walking a real
// packet means striding by
// sizeof(LegacyMeasurementBlockHeader) + pixels_per_column*sizeof(LegacyChannelBlock) + 4
// per block, not treating the whole packet as one fixed struct.
//
// Relies on host == wire endianness (little-endian) -- true for x86 and
// ARM, which is all this project targets.

#pragma pack(push, 1)

struct LegacyMeasurementBlockHeader {
  union {
    struct {
      uint32_t timestamp_low;
      uint32_t timestamp_high;
    };
    uint64_t timestamp_ns;
  };
  uint16_t measurement_id;
  uint16_t frame_id;
  uint32_t encoder_count : 24;
  uint32_t reserved      : 8;
};
static_assert(sizeof(LegacyMeasurementBlockHeader) == 16, "LegacyMeasurementBlockHeader must be 16 bytes");

struct LegacyChannelBlock {
  uint32_t range      : 20;
  uint32_t reserved1  : 8;
  uint32_t bloom      : 1;
  uint32_t reserved2  : 3;
  uint8_t  reflectivity;
  uint8_t  reserved3;
  uint16_t signal;
  uint16_t near_ir;
  uint16_t reserved4;
};
static_assert(sizeof(LegacyChannelBlock) == 12, "LegacyChannelBlock must be 12 bytes");

struct LegacyIMUPacket {
  uint64_t timestamp_ns;
  uint64_t acc_timestamp_ns;
  uint64_t gyro_timestamp_ns;
  float    accel_x;
  float    accel_y;
  float    accel_z;
  float    gyro_x;
  float    gyro_y;
  float    gyro_z;
};
static_assert(sizeof(LegacyIMUPacket) == 48, "LegacyIMUPacket must be 48 bytes");
#pragma pack(pop)
