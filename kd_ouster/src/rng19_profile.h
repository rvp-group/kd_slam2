#pragma once
#include "ouster_lidar.h"

// RNG19_RFL8_SIG16_NIR16 single-return profile. IMU stays LEGACY on every
// sensor seen so far, so there's no Rng19ImuProfile -- LegacyImuProfile
// covers it.
struct Rng19LidarProfile: public OusterLidar::LidarProfile {
  Rng19LidarProfile(OusterLidar& oust, int columns_per_packet, int pixels_per_column);
  int parsePacket(const uint8_t* p, size_t sz) override;
};
