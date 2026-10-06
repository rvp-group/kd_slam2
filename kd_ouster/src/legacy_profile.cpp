#include "legacy_profile.h"
#include "ouster_legacy_packets.h"

LegacyLidarProfile::LegacyLidarProfile(OusterLidar& oust, int columns_per_packet, int pixels_per_column):
  LidarProfile(oust,
               columns_per_packet*(sizeof(LegacyMeasurementBlockHeader)
                                    +sizeof(uint32_t)+pixels_per_column*sizeof(LegacyChannelBlock)),
               1e-3f) {}

int LegacyLidarProfile::parsePacket(const uint8_t* packet, size_t size) {
  if (size!=_packet_size)
    return -1;
  auto& oust=_ouster;
  const uint8_t* bstart=packet;
  bool new_cloud=false;
  double column_stamp=0;
  for (int i=0; i<oust.columns_per_packet; ++i) {
    const LegacyMeasurementBlockHeader* header=reinterpret_cast<const LegacyMeasurementBlockHeader*>(bstart);
    bstart+=sizeof(LegacyMeasurementBlockHeader);
    const uint8_t* b_end=bstart+ oust.pixels_per_column *sizeof(LegacyChannelBlock)+sizeof(uint32_t);
    const uint32_t* status=reinterpret_cast<const uint32_t*>(bstart+ oust.pixels_per_column *sizeof(LegacyChannelBlock));
    if (header->frame_id!=oust.prev_frame_id) {
      if (oust.frames_received>=2) {
        new_cloud=true;
        oust.cloud_idx=1-oust.cloud_idx;
      }
      ++oust.frames_received;
      oust.pendingCloud().points.clear();
      oust.prev_frame_id=header->frame_id;
    }
    column_stamp=(double)header->timestamp_ns*1e-9;
    if (new_cloud) {
      oust.pendingCloud().header.stamp_ns=header->timestamp_ns;
    }
    if ((*status)==0xffffffff) {
      updateColumn(header->measurement_id);
      for (int c=0; c< oust.pixels_per_column; ++c) {
        const LegacyChannelBlock* ray=reinterpret_cast<const LegacyChannelBlock*>(bstart);
        bstart+=sizeof(LegacyChannelBlock);
        OusterLidar::Point p=toPoint(column_stamp, c, ray->range, ray->reflectivity, ray->signal, ray->near_ir);
        //oust.pendingCloud().points.push_back(p);
        oust.pendingCloud().points.push_back(p);
      }
      oust.valid_columns++;
    } else
      oust.dropped_columns++;
    bstart=b_end;
  }
  if (! new_cloud)
    return 0;
  return 1;
}

LegacyImuProfile::LegacyImuProfile(OusterLidar& oust):
  Profile(oust, sizeof(LegacyIMUPacket)) {}

int LegacyImuProfile::parsePacket(const uint8_t* packet, size_t size) {
  if (size!=_packet_size)
    return -1;
  const LegacyIMUPacket& ip=*reinterpret_cast<const LegacyIMUPacket*>(packet);
  _ouster.imu_idx=1-_ouster.imu_idx;
  auto& o=_ouster.pendingIMU();
  o.stamp=(double)ip.timestamp_ns*1e-9;
  o.acc_stamp=(double)ip.acc_timestamp_ns*1e-9;
  o.gyro_stamp=(double)ip.gyro_timestamp_ns*1e-9;
  o.accel_x=ip.accel_x;
  o.accel_y=ip.accel_y;
  o.accel_z=ip.accel_z;
  o.gyro_x=ip.gyro_x;
  o.gyro_y=ip.gyro_y;
  o.gyro_z=ip.gyro_z;
  return 1;
}
