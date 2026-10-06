#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include "ouster_lidar.h"

using namespace std;

constexpr size_t kMaxCloudsToDump = 10;

int main(int argc, char** argv) {
  if (argc < 2) {
    cerr << "usage: " << argv[0] << " <session.json>" << endl;
    return -1;
  }

  OusterLidar oust;
  size_t clouds_seen = 0;
  size_t points_seen = 0;
  size_t imus_seen = 0;
  size_t packets_seen = 0;

  oust.on_cloud = [&](const OusterLidar::CloudMessageData& c) {
    ++clouds_seen;
    //points_seen += c.points.size();
    points_seen += c.points.size();
    cerr << "cloud #" << clouds_seen << " stamp=" << c.header.stamp_ns
         << " points=" << c.points.size() << endl;
    if (clouds_seen <= kMaxCloudsToDump) {
      ostringstream name;
      name << "cloud_" << setw(4) << setfill('0') << clouds_seen << ".txt";
      ofstream os(name.str());
      OusterLidar::print(os,c);
    }
  };
  oust.on_imu = [&](const OusterLidar::IMUData& i) {
    ++imus_seen;
    cerr << "imu #" << imus_seen << " stamp=" << i.stamp << endl;
  };
  oust.on_packet = [&](uint32_t, uint16_t, const uint8_t*, size_t) {
    ++packets_seen;
  };

  int ok = oust.open(argv[1]);
  if (ok != 1) {
    cerr << "open failed: " << ok << endl;
    return -1;
  }

  oust.spinLog();

  cerr << "done: " << packets_seen << " packets, "
       << clouds_seen << " clouds (" << points_seen << " points), "
       << imus_seen << " imu samples" << endl;
}
