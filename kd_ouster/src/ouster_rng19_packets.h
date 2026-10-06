#pragma once
#include <cstdint>

// RNG19_RFL8_SIG16_NIR16 lidar UDP packet layout (single-return profile),
// verified against a real captured packet from an OS1-64.
//
// A packet is: PacketHeader, then columns_per_packet x
// [Rng19ColumnHeader + pixels_per_column x Rng19ChannelBlock], then
// PacketFooter. pixels_per_column is a runtime value from the sensor's
// metadata, not fixed here.
//
// Unlike LEGACY, frame_id lives once per packet (PacketHeader), not
// repeated per column.
//
// Relies on host == wire endianness (little-endian) -- true for x86 and
// ARM, which is all this project targets.

#pragma pack(push, 1)

struct PacketHeader {
  uint16_t packet_type;
  uint16_t frame_id;
  uint8_t  reserved0[28];
};
static_assert(sizeof(PacketHeader) == 32, "PacketHeader must be 32 bytes");

struct Rng19ColumnHeader {
  uint64_t timestamp_ns;
  uint16_t measurement_id;
  uint16_t status;  // bit0 = valid
};
static_assert(sizeof(Rng19ColumnHeader) == 12, "Rng19ColumnHeader must be 12 bytes");

struct Rng19ChannelBlock {
  uint32_t range      : 19;
  uint32_t reserved1  : 13;
  uint8_t  reflectivity;
  uint8_t  reserved2;
  uint16_t signal;
  uint16_t near_ir;
  uint8_t  reserved3;
  uint8_t  window;
};
static_assert(sizeof(Rng19ChannelBlock) == 12, "Rng19ChannelBlock must be 12 bytes");

struct PacketFooter {
  uint8_t  reserved[24];
  uint64_t crc64;
};
static_assert(sizeof(PacketFooter) == 32, "PacketFooter must be 32 bytes");

#pragma pack(pop)
