#include "kd_slam/utils/time_utils.h"
using namespace kd_slam::utils;
#include <iomanip>
#include <iostream>
#include <string>
#include "srrg_system_utils/parse_command_line.h"
using namespace srrg2_core;
#include "kd_io/slam_messages.h"
#include "kd_io/mcap_message_reader.h"
#include "kd_io/mcap_writer.h"
#include "kd_slam/descriptor/descriptor_.h"
#include "kd_slam/descriptor/extractor_.h"
using namespace kd_slam;
using namespace kd_slam::descriptor;

// Extract descriptors from a KDTree MCAP and write them as KDDescriptorData MCAP.
// Each input tree message produces NumAxesCanonizations output messages (one per canonization).

template <typename KDTreeDataT, int Level>
void run(const std::string& in_bag,
         const std::string& topic,
         const std::string& out_bag,
         const std::string& out_topic) {
  using namespace std;
  using TreeType  = decltype(KDTreeDataT::tree);
  using Extractor = Extractor_<TreeType, Level>;
  using Desc      = typename Extractor::DescriptorType;
  static constexpr int NumCan = Extractor::NumAxesCanonizations;
  static constexpr int Dim    = Extractor::Dim;
  static constexpr int Size   = Extractor::Size;

  cerr << "dim=" << Dim << "  level=" << Level
       << "  size=" << Size << "  num_canonizations=" << NumCan << "\n"
       << "input:  " << in_bag  << " [" << topic     << "]\n"
       << "output: " << out_bag << " [" << out_topic << "]\n\n";

  McapWriter writer(out_bag);
  uint16_t ch_id = writer.addChannel<KDDescriptorData>(out_topic);

  int total = 0, skipped = 0;
  auto t0 = getNow();

  McapMessageReader reader;
  reader.registerHandler<KDTreeDataT>(topic);
  reader.open(in_bag);
  while (reader.isGood()) {
    auto msg = reader.readOne<KDTreeDataT>();
    if (!msg) continue;
    ++total;
    if (msg->tree.numNodes() < Size + 1) {
      cerr << "tree " << total << ": too few nodes (" << msg->tree.numNodes() << "), skipping\n";
      ++skipped;
      continue;
    }
    try {
      for (int can = 0; can < NumCan; ++can) {
        Desc desc = Extractor::extract(msg->tree, can);
        KDDescriptorData dmsg = toMessage(desc, msg->tree.root_eigenvectors,
                                          msg->header, topic, Level);
        writer.write(ch_id, msg->header.stamp_ns, dmsg);
      }
    } catch (const std::exception& e) {
      cerr << "tree " << total << ": extract failed (" << e.what() << "), skipping\n";
      ++skipped;
      continue;
    }
    cerr << "  tree " << total
         << " | elapsed: " << fixed << setprecision(1)
         << (getDurationMs(t0)) * 1e-3 << " s    \r";
  }
  writer.close();

  cerr << "\n\nDone.  Trees=" << total
       << "  Skipped=" << skipped
       << "  Descriptors written=" << (total - skipped) * NumCan
       << "  Elapsed=" << fixed << setprecision(1) << (getDurationMs(t0)) * 1e-3 << " s\n";
}

const char* kdtree2descriptor_banner[] = {
  "kdtree2descriptor: extract PCA KD-tree descriptors from a KDTree MCAP bag",
  "usage: kdtree2descriptor [options] <input.mcap> <topic>",
  0};

int main(int argc, char** argv) {
  using namespace std;
  ParseCommandLine cmd_line(argv, kdtree2descriptor_banner);
  ArgumentString a_out      (&cmd_line, "o", "output",    "output MCAP file",          "out_descriptors.mcap");
  ArgumentString a_out_topic(&cmd_line, "t", "out-topic", "output MCAP topic",         "/descriptors");
  ArgumentFlag   a_2d       (&cmd_line, "2", "2d",        "input trees are 2D (default: 3D)");
  cmd_line.parse();

  const auto& pos_args = cmd_line.lastParsedArgs();
  if (pos_args.size() < 2) { cerr << cmd_line.options(); return 1; }

  const string in_bag    = pos_args[0];
  const string topic     = pos_args[1];
  const string out_bag   = a_out.value();
  const string out_topic = a_out_topic.value();
  const bool   is_2d     = a_2d.isSet();

  try {
    if (is_2d)
      run<KDTreeData2f,5>(in_bag, topic, out_bag, out_topic);
    else
      run<KDTreeData3f,5>(in_bag, topic, out_bag, out_topic);
  } catch (const exception& e) {
    cerr << "\nError: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
