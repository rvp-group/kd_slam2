// Converts livox_ros_driver2/msg/CustomMsg bag or MCAP to sensor_msgs/msg/PointCloud2 MCAP.
// IMU is passed through unchanged.
//
// Usage: kd_livox_converter -i <input> -o <output.mcap> [options]
// Input can be a .mcap file or a ROS2 bag directory.

#include <cmath>
#include <iostream>
#include <memory>
#include <string>

#include "kd_io/rosbag2_message_reader.h"
#include "kd_io/rosbag2_message_writer.h"
#include "kd_io/slam_messages.h"

static void printUsage(const char* prog) {
  std::cerr << "Usage: " << prog
            << " -i <input> -o <output.mcap>"
            << " [-tl <lidar-topic>] [-ti <imu-topic>] [-to <out-cloud-topic>]\n"
            << "  -tl  Livox lidar topic  (default: /livox/lidar)\n"
            << "  -ti  IMU topic          (default: /livox/imu, empty = skip)\n"
            << "  -to  output cloud topic (default: /points)\n";
}

static std::string getArg(int argc, char** argv, const char* flag, const char* def) {
  for (int i = 1; i + 1 < argc; ++i)
    if (std::string(argv[i]) == flag) return argv[i + 1];
  return def;
}

static bool hasFlag(int argc, char** argv, const char* flag) {
  for (int i = 1; i < argc; ++i)
    if (std::string(argv[i]) == flag) return true;
  return false;
}

int main(int argc, char** argv) {
  if (hasFlag(argc, argv, "-h") || !hasFlag(argc, argv, "-i") || !hasFlag(argc, argv, "-o")) {
    printUsage(argv[0]);
    return 1;
  }

  std::string in_path   = getArg(argc, argv, "-i",  "");
  std::string out_path  = getArg(argc, argv, "-o",  "");
  std::string lidar_top = getArg(argc, argv, "-tl", "/livox/lidar");
  std::string imu_top   = getArg(argc, argv, "-ti", "/livox/imu");
  std::string out_cloud = getArg(argc, argv, "-to", "/points");

  // The Livox custom message type is not registered in our type table,
  // so we read it raw via the rosbag2 API and convert manually.
  // For now this converter requires the bag to contain sensor_msgs/PointCloud2
  // (already converted) or a Livox bag processed through another tool first.
  // TODO: add native Livox CDR deserialization.

  Rosbag2MessageReader reader;
  reader.param_bag_path.setValue(in_path);
  reader.param_topics.value().push_back(lidar_top);
  if (!imu_top.empty()) reader.param_topics.value().push_back(imu_top);
  reader.open();

  Rosbag2MessageWriter writer;
  writer.param_out_path.setValue(out_path);
  writer.open();

  int n_clouds = 0, n_imus = 0;

  while (reader.isGood()) {
    auto anon = reader.readOne();
    if (!anon) continue;

    if (auto cloud = std::dynamic_pointer_cast<Message_<PointCloudXYZTData>>(anon)) {
      writer.write(out_cloud, cloud->header.stamp_ns, *cloud);
      ++n_clouds;
      if (n_clouds % 200 == 0)
        std::cerr << "\rclouds: " << n_clouds << "  imus: " << n_imus << "   ";
      continue;
    }

    if (!imu_top.empty()) {
      if (auto imu = std::dynamic_pointer_cast<Message_<ImuData>>(anon)) {
        writer.write(imu_top, imu->header.stamp_ns, *imu);
        ++n_imus;
        continue;
      }
    }
  }

  std::cerr << "\ndone: " << n_clouds << " clouds, " << n_imus << " imus\n";
  writer.close();
  return 0;
}
