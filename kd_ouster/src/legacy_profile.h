#pragma once
#include "ouster_lidar.h"

struct LegacyLidarProfile: public OusterLidar::LidarProfile {
  LegacyLidarProfile(OusterLidar& oust, int columns_per_packet, int pixels_per_column);
  int parsePacket(const uint8_t* p, size_t sz) override;
};

struct LegacyImuProfile: public OusterLidar::Profile {
  explicit LegacyImuProfile(OusterLidar& oust);
  int parsePacket(const uint8_t* p, size_t sz) override;
};
