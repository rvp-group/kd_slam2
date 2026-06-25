// Converts a zuppo_log.txt (Teensy text protocol) to an MCAP file
// with PointCloudXYTData messages, one per lidar revolution.
//
// Usage: zuppo_to_mcap <input.txt> <output.mcap>
//
// LIDAR line format:
//   LIDAR <ts_us> <id> <angle_start_centideg> <angle_end_centideg> <speed_rpm>
//         [<dist_mm> <intensity>] x 12

#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "kd_io/mcap_writer.h"
#include "kd_io/slam_messages.h"

static constexpr float kMinRangeM   = 0.05f;
static constexpr float kMaxRangeM   = 20.0f;
static constexpr int   kBeamsPerPkt = 12;

struct LidarAccum {
  std::vector<PointXYT> points;
  int    max_angle  = -1;
  double t_start_s  = 0.0;
  bool   started    = false;
};

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "usage: zuppo_to_mcap <input.txt> <output.mcap>\n";
    return 1;
  }
  std::ifstream in(argv[1]);
  if (!in) { std::cerr << "cannot open " << argv[1] << "\n"; return 1; }

  McapWriter writer(argv[2]);

  std::map<int, uint16_t>   channels;
  std::map<int, LidarAccum> accum;
  int scan_count = 0;

  auto flush = [&](int id, LidarAccum& acc) {
    if (acc.points.empty()) return;
    if (channels.find(id) == channels.end())
      channels[id] = writer.addChannel<PointCloudXYTData>("/lidar_" + std::to_string(id));
    PointCloudXYTData msg;
    msg.header.stamp_ns          = static_cast<uint64_t>(acc.t_start_s * 1e9);
    msg.header.frame_id          = "lidar_" + std::to_string(id);
    msg.has_per_point_timestamps = true;
    msg.points                   = std::move(acc.points);
    writer.write(channels[id], msg.header.stamp_ns, msg);
    ++scan_count;
    acc.points.clear();
    acc.max_angle = -1;
    acc.started   = false;
  };

  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ss(line);
    std::string tag;
    ss >> tag;
    if (tag != "LIDAR") continue;

    uint64_t ts_us;
    int id, angle_start, angle_end, speed_rpm;
    ss >> ts_us >> id >> angle_start >> angle_end >> speed_rpm;
    double t_s = ts_us * 1e-6;

    LidarAccum& acc = accum[id];

    int span = angle_end - angle_start;
    if (span < 0) span += 36000;

    // Detect revolution wraparound
    if (acc.started && (angle_start < acc.max_angle || angle_end < angle_start))
      flush(id, acc);

    if (!acc.started) {
      acc.t_start_s = t_s;
      acc.started   = true;
    }

    int d, intensity;
    for (int i = 0; i < kBeamsPerPkt; ++i) {
      if (!(ss >> d >> intensity)) break;
      if (d == 0) continue;
      float r = d * 1e-3f;
      if (r < kMinRangeM || r > kMaxRangeM) continue;
      float angle_centideg = angle_start + (float)i * span / kBeamsPerPkt;
      float angle_rad      = angle_centideg * (float)(M_PI / 18000.0);
      PointXYT pt;
      pt.xy = Eigen::Vector2f(r * std::cos(angle_rad), r * std::sin(angle_rad));
      pt.t  = t_s;
      acc.points.push_back(pt);
    }
    acc.max_angle = angle_end;
  }

  for (auto& [id, acc] : accum)
    flush(id, acc);

  writer.close();
  std::cout << scan_count << " scans written to " << argv[2] << "\n";
  return 0;
}
