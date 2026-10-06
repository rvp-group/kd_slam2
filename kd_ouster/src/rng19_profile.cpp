#include "rng19_profile.h"
#include "ouster_rng19_packets.h"

Rng19LidarProfile::Rng19LidarProfile(OusterLidar& oust, int columns_per_packet, int pixels_per_column):
  LidarProfile(oust,
               sizeof(PacketHeader)
                 +columns_per_packet*(sizeof(Rng19ColumnHeader)+pixels_per_column*sizeof(Rng19ChannelBlock))
                 +sizeof(PacketFooter),
               1e-3f) {}

int Rng19LidarProfile::parsePacket(const uint8_t* packet, size_t size) {
  if (size!=_packet_size)
    return -1;
  auto& oust=_ouster;
  // unlike LEGACY, frame_id lives once per packet, not repeated per column
  const PacketHeader* pheader=reinterpret_cast<const PacketHeader*>(packet);
  const uint8_t* bstart=packet+sizeof(PacketHeader);
  bool new_cloud=false;
  if (pheader->frame_id!=oust.prev_frame_id) {
    if (oust.frames_received>=2) {
      new_cloud=true;
      oust.cloud_idx=1-oust.cloud_idx;
    }
    ++oust.frames_received;
    //oust.pendingCloud().reset();
    oust.pendingCloud().points.clear();
    oust.prev_frame_id=pheader->frame_id;
  }
  double column_stamp=0;
  for (int i=0; i<oust.columns_per_packet; ++i) {
    const Rng19ColumnHeader* header=reinterpret_cast<const Rng19ColumnHeader*>(bstart);
    bstart+=sizeof(Rng19ColumnHeader);
    const uint8_t* b_end=bstart+ oust.pixels_per_column *sizeof(Rng19ChannelBlock);
    column_stamp=(double)header->timestamp_ns*1e-9;
    // if (i==0 && new_cloud) {
    //   oust.pendingCloud().stamp=column_stamp;
    // }
    if (header->status & 0x1) {
      updateColumn(header->measurement_id);
      for (int c=0; c< oust.pixels_per_column; ++c) {
        const Rng19ChannelBlock* ray=reinterpret_cast<const Rng19ChannelBlock*>(bstart);
        bstart+=sizeof(Rng19ChannelBlock);
        OusterLidar::Point p=toPoint(column_stamp, c, ray->range, ray->reflectivity, ray->signal, ray->near_ir);
        //oust.pendingCloud().points.push_back(p);
        oust.pendingCloud().points.push_back(p);
      }
    }
    bstart=b_end;
  }
  if (! new_cloud)
    return 0;
  return 1;
}
