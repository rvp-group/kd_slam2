#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/time.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>

namespace fs = std::filesystem;

// kitti2ros2bag <seq_dir> <output.mcap> [topic]
//   seq_dir : KITTI sequence directory (velodyne/ + times.txt, or velodyne_points/data/ + timestamps.txt)
//   output  : output mcap bag
//   topic   : point cloud topic (default /velodyne_points)

static uint64_t parseRawTimestamp(const std::string& line) {
  int yr, mo, dy, hh, mm, ss;
  if (sscanf(line.c_str(), "%d-%d-%d %d:%d:%d", &yr, &mo, &dy, &hh, &mm, &ss) != 6)
    return 0;
  struct tm t = {};
  t.tm_year = yr - 1900; t.tm_mon = mo - 1; t.tm_mday = dy;
  t.tm_hour = hh; t.tm_min = mm; t.tm_sec = ss; t.tm_isdst = -1;
  time_t epoch = mktime(&t);
  char frac[10] = "000000000";
  const char* dot = strchr(line.c_str(), '.');
  if (dot) { strncpy(frac, dot + 1, 9); frac[9] = '\0'; for (size_t i = strlen(frac); i < 9; ++i) frac[i] = '0'; }
  return static_cast<uint64_t>(epoch) * 1000000000ULL + static_cast<uint64_t>(atoll(frac));
}

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "usage: kitti2ros2bag <seq_dir> <output.mcap> [topic]\n";
    return 1;
  }
  const std::string seq_dir  = argv[1];
  const std::string out_path = argv[2];
  const std::string topic    = (argc > 3) ? argv[3] : "/velodyne_points";

  fs::path base(seq_dir);
  fs::path velo_dir, times_path;
  bool raw_format = false;

  if (fs::is_directory(base / "velodyne_points" / "data")) {
    velo_dir   = base / "velodyne_points" / "data";
    times_path = base / "velodyne_points" / "timestamps.txt";
    raw_format = true;
  } else if (fs::is_directory(base / "velodyne")) {
    velo_dir   = base / "velodyne";
    times_path = base / "times.txt";
  } else {
    std::cerr << "neither velodyne_points/data/ nor velodyne/ found in " << seq_dir << "\n";
    return 1;
  }

  std::vector<std::string> bin_files;
  for (const auto& e : fs::directory_iterator(velo_dir))
    if (e.path().extension() == ".bin")
      bin_files.push_back(e.path().string());
  std::sort(bin_files.begin(), bin_files.end());

  std::vector<uint64_t> times_ns;
  std::ifstream tf(times_path);
  if (!tf.good()) { std::cerr << "cannot open: " << times_path << "\n"; return 1; }
  std::string line;
  while (std::getline(tf, line)) {
    if (line.empty()) continue;
    times_ns.push_back(raw_format ? parseRawTimestamp(line)
                                  : static_cast<uint64_t>(std::stod(line) * 1e9));
  }

  if (bin_files.size() != times_ns.size()) {
    std::cerr << "bin count (" << bin_files.size() << ") != times count (" << times_ns.size() << ")\n";
    return 1;
  }

  rosbag2_cpp::Writer writer;
  rosbag2_storage::StorageOptions storage_opts;
  storage_opts.uri            = out_path;
  storage_opts.storage_id     = "mcap";
  writer.open(storage_opts);

  rosbag2_cpp::ConverterOptions conv_opts;
  conv_opts.input_serialization_format  = "cdr";
  conv_opts.output_serialization_format = "cdr";

  rosbag2_storage::TopicMetadata meta;
  meta.name            = topic;
  meta.type            = "sensor_msgs/msg/PointCloud2";
  meta.serialization_format = "cdr";
  writer.create_topic(meta);

  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;

  for (size_t i = 0; i < bin_files.size(); ++i) {
    FILE* fp = fopen(bin_files[i].c_str(), "rb");
    if (!fp) continue;
    fseek(fp, 0, SEEK_END);
    size_t n = static_cast<size_t>(ftell(fp)) / (4 * sizeof(float));
    rewind(fp);

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp.sec     = static_cast<int32_t>(times_ns[i] / 1000000000ULL);
    cloud.header.stamp.nanosec = static_cast<uint32_t>(times_ns[i] % 1000000000ULL);
    cloud.header.frame_id      = "velodyne";
    cloud.height    = 1;
    cloud.width     = static_cast<uint32_t>(n);
    cloud.is_dense  = true;
    cloud.is_bigendian = false;
    cloud.point_step   = 4 * sizeof(float);
    cloud.row_step     = cloud.point_step * cloud.width;

    for (const auto& [name, offset] : std::vector<std::pair<std::string,uint32_t>>{{"x",0},{"y",4},{"z",8},{"intensity",12}}) {
      sensor_msgs::msg::PointField f;
      f.name     = name;
      f.offset   = offset;
      f.datatype = sensor_msgs::msg::PointField::FLOAT32;
      f.count    = 1;
      cloud.fields.push_back(f);
    }

    cloud.data.resize(cloud.row_step);
    fread(cloud.data.data(), sizeof(float), 4 * n, fp);
    fclose(fp);

    auto bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    bag_msg->topic_name  = topic;
    bag_msg->recv_timestamp = static_cast<rcutils_time_point_value_t>(times_ns[i]);
    bag_msg->send_timestamp = bag_msg->recv_timestamp;
    rclcpp::SerializedMessage serialized_msg;
    serializer.serialize_message(&cloud, &serialized_msg);
    bag_msg->serialized_data = std::make_shared<rcutils_uint8_array_t>();
    *bag_msg->serialized_data = serialized_msg.release_rcl_serialized_message();
    writer.write(bag_msg);

    if ((i + 1) % 100 == 0)
      std::cerr << "\r" << (i + 1) << "/" << bin_files.size() << " frames" << std::flush;
  }
  std::cerr << "\ndone: " << bin_files.size() << " frames -> " << out_path << "\n";
  return 0;
}
