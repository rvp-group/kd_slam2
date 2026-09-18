#pragma once
#include "tree_loader_.h"
#include "kd_slam/tree/tree_generator_impl_.h"
#include "kd_slam/utils/voxelizer_impl_.h"
#include "kd_slam/utils/time_utils.h"
#include "kd_io/scan_utils_.h"
#include <tf2/buffer_core.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace kd_slam {
  using namespace std;

 

  template <typename NodeType_>
  std::shared_ptr<typename TreeLoader_<NodeType_>::FrameTree>
  TreeLoader_<NodeType_>::getFrame(const std::string& topic,
                                   uint64_t offset,
                                   uint64_t stamp_ns) {
    auto reader = param_reader.value();
    if (!reader) return nullptr;
    if (!reader->isOpen()) reader->open();
    auto anon_msg = reader->readOne(topic, offset, stamp_ns);
    if (!anon_msg) return nullptr;
    if (auto msg = std::dynamic_pointer_cast<Message_<PointCloudDataType>>(anon_msg))
      return fromCloudMsg(msg);
    if (auto msg = std::dynamic_pointer_cast<Message_<TreeDataType>>(anon_msg))
      return fromTreeMsg(msg);
    return nullptr;
  }

  template <typename NodeType_>
  std::shared_ptr<typename TreeLoader_<NodeType_>::FrameTree>
  TreeLoader_<NodeType_>::fromInternalCloudMsg(std::shared_ptr<Message_<TreeLoader_<NodeType_>::InternalCloudData>> msg) {
    using namespace std;
    double ts = msg->header.stamp_ns * 1e-9;
    auto generator = param_generator.value();
    auto writer    = param_writer.value();

    auto& pts = msg->points;
    if (pts.empty()) return nullptr;
    if (on_cloud) on_cloud(pts, ts);
    if (!generator) return nullptr; 
    auto t_tree = utils::getNow();
    auto tree_cpu = generator->makeTree(pts);
    t_tree_ms += utils::getDurationMs(t_tree);
    if (!tree_cpu) return nullptr;
    auto frame        = std::make_shared<FrameTree>();
    frame->ts         = ts;
    frame->stamp_ns   = msg->log_stamp_ns;
    frame->bag_offset = msg->file_offset;
    frame->tree       = tree_cpu;
    frame->topic      = msg->topic;
    if (writer) {
      Message_<TreeDataType> wrapped;
      static_cast<TreeDataType&>(wrapped) = toMessage(*tree_cpu, msg->header, msg->topic);
      writer->write(param_out_topic.value(), msg->header.stamp_ns, wrapped);
    }
    return frame;
  }

  template <typename NodeType_>
  std::shared_ptr<typename TreeLoader_<NodeType_>::FrameTree>
  TreeLoader_<NodeType_>::fromCloudMsg(std::shared_ptr<Message_<PointCloudDataType>> msg) {
    using namespace std;
    double ts = msg->header.stamp_ns * 1e-9;
    auto voxelizer = param_voxelizer.value();
    auto generator = param_generator.value();
    auto writer    = param_writer.value();

    auto pts = toPointsVector<PointTraits>(static_cast<const PointCloudDataType&>(*msg), -1);
    if (pts.empty()) return nullptr;

    auto t_vox = utils::getNow();
    
    if (voxelizer)
      pts = voxelizer->voxelize(pts);
    t_vox_ms += utils::getDurationMs(t_vox);
    if (pts.empty()) return nullptr;

    if (on_cloud) on_cloud(pts, ts);
    if (!generator) return nullptr; 
    auto t_tree = utils::getNow();
    auto tree_cpu = generator->makeTree(pts);
    t_tree_ms += utils::getDurationMs(t_tree);
    if (!tree_cpu) return nullptr;
    auto frame        = std::make_shared<FrameTree>();
    frame->ts         = ts;
    frame->stamp_ns   = msg->log_stamp_ns;
    frame->bag_offset = msg->file_offset;
    frame->tree       = tree_cpu;
    frame->topic      = msg->topic;
    if (writer) {
      Message_<TreeDataType> wrapped;
      static_cast<TreeDataType&>(wrapped) = toMessage(*tree_cpu, msg->header, msg->topic);
      writer->write(param_out_topic.value(), msg->header.stamp_ns, wrapped);
    }
    return frame;
  }

  template <typename NodeType_>
  std::shared_ptr<typename TreeLoader_<NodeType_>::FrameTree>
  TreeLoader_<NodeType_>::fromTreeMsg(std::shared_ptr<Message_<TreeDataType>> msg) {
    auto frame        = std::make_shared<FrameTree>();
    frame->ts         = msg->header.stamp_ns * 1e-9;
    frame->stamp_ns   = msg->log_stamp_ns;
    frame->bag_offset = msg->file_offset;
    frame->tree       = std::make_shared<TreeType>(msg->tree);
    frame->topic      = msg->topic;
    return frame;
  }

  template <typename NodeType_>
  TreeLoader_<NodeType_>::~TreeLoader_() = default;

  template <typename NodeType_>
  void TreeLoader_<NodeType_>::_tryResolveTF() {
    if (_tf_resolved) return;
    if (_base_frame.empty() || _lidar_frame.empty()) return;
    auto lookup = [&](const std::string& target, const std::string& source,
                      Eigen::Isometry3d& out) {
      try {
        auto ts = _tf_buffer->lookupTransform(target, source, tf2::TimePointZero);
        out.translation() = Eigen::Vector3d(ts.transform.translation.x,
                                            ts.transform.translation.y,
                                            ts.transform.translation.z);
        out.linear() = Eigen::Quaterniond(ts.transform.rotation.w,
                                          ts.transform.rotation.x,
                                          ts.transform.rotation.y,
                                          ts.transform.rotation.z).toRotationMatrix();
      } catch (const tf2::LookupException& e) {
        std::cerr << "[TreeLoader_] TF lookup " << source << "->" << target
                  << " failed: " << e.what() << " -- using identity\n";
      }
    };
    lookup(_base_frame, _lidar_frame, _T_base_lidar);
    if (!_imu_frame.empty())
      lookup(_base_frame, _imu_frame, _T_base_imu);
    _tf_resolved = true;
  }

  template <typename NodeType_>
  void TreeLoader_<NodeType_>::run(bool rn) {
    auto reader = param_reader.value();
    auto writer = param_writer.value();
    if (!reader) {
      std::cerr << "[TreeLoader_] no reader configured\n";
      return;
    }
    if (!_frame_queue) {
      reader->open();
      if (writer) writer->open();
      _runner();
      return;
    }
    if (!_runner_thread.joinable()) {
      _runner_thread = std::thread([this, reader, writer]() {
        pthread_setname_np(pthread_self(), "preprocessor");
        reader->open();
        if (writer) writer->open();
        _runner();
      });
      Runnable::run(rn);
    } else {
      Runnable::run(rn);
    }
  }

  template <int Dim, typename Scalar_, typename SrcType>
  Eigen::Transform<Scalar_, Dim, Eigen::Isometry> transformIso_(const SrcType& src) {
    if constexpr (Dim==3)
      return src.template cast<Scalar_>();
    else {
      Eigen::Transform<Scalar_, Dim, Eigen::Isometry> ret;
      ret.translation()=src.translation().template head<Dim>().template cast <Scalar_>();
      ret.linear()=src.linear().template  block<Dim, Dim>(0,0).template cast<Scalar_>();
      return ret;
    }
  }

  template <typename NodeType_>
  void TreeLoader_<NodeType_>::_reader_runner(){
    pthread_setname_np(pthread_self(), "reader");
    auto reader  = param_reader.value();
    t_read_ms=0;
    while (reader->isGood()) {
      auto t0 = utils::getNow();
      auto anon_msg = reader->readOne();
      t_read_ms += utils::getDurationMs(t0);
      if (!anon_msg) continue;
      _reader_queue.push(anon_msg);
    }
    _reader_queue.push(nullptr); 
   _reader_queue.setDone();
  }

  template <typename NodeType_>
  void TreeLoader_<NodeType_>::_voxelizer_runner(){
    pthread_setname_np(pthread_self(), "voxelizer");
    auto voxelizer = param_voxelizer.value();
    t_vox_ms=0;
    std::shared_ptr<MessageBase> anon_msg;
    while ((anon_msg=_reader_queue.pop())) {
      if (auto msg = std::dynamic_pointer_cast<Message_<PointCloudDataType>>(anon_msg)) {
        auto out_msg=std::shared_ptr<Message_<InternalCloudData> > (new Message_<InternalCloudData>);
        out_msg->header=msg->header;
        if (msg->points.empty()) {
          _voxelizer_queue.push(out_msg);
          continue;
        }
        out_msg->points = toPointsVector<PointTraits>(static_cast<const PointCloudDataType&>(*msg), -1);
        auto t_vox = utils::getNow();
        if (voxelizer)
          out_msg->points = voxelizer->voxelize(out_msg->points);
        t_vox_ms += utils::getDurationMs(t_vox);
        anon_msg=out_msg;
      } 
      _voxelizer_queue.push(anon_msg);
    }
    _voxelizer_queue.push(nullptr); 
    _voxelizer_queue.setDone();
  }

  template <typename NodeType_>
  void TreeLoader_<NodeType_>::_runner() {
    std::thread reader_thread(&TreeLoader_<NodeType_>::_reader_runner, this);
    std::thread voxelizer_thread(&TreeLoader_<NodeType_>::_voxelizer_runner, this);
    _tf_buffer = std::make_unique<tf2::BufferCore>();
    auto writer  = param_writer.value();
    bool verbose = param_verbose.value();

    int num_clouds = 0, num_trees = 0, num_imus = 0, num_odoms = 0, num_tfs = 0;
    std::map<double, ImuData> imus;
    std::map<double, OdometryData> odometries;
    std::shared_ptr<MessageBase> anon_msg;
    while ((anon_msg=_voxelizer_queue.pop())) {
      Runnable::waitRun();
      if (verbose) {
        cerr << "\rclouds: " << num_clouds
             << " imus: "   << num_imus
             << " odom: "   << num_odoms
             << " trees: "  << num_trees
             << " tfs: "    << num_tfs
             << " avg_read_ms: " << (num_clouds ? t_read_ms/num_clouds : 0)
             << " avg_vox_ms: " << (num_clouds ? t_vox_ms/num_clouds : 0)
             << " avg_tree_ms: " << (num_clouds ? t_tree_ms/num_clouds : 0);
      }
      if (auto msg = std::dynamic_pointer_cast<Message_<TFMessageData>>(anon_msg)) {
        ++num_tfs;
        for (const auto& td : msg->transforms) {
          geometry_msgs::msg::TransformStamped ts;
          ts.header.frame_id      = td.header.frame_id;
          ts.header.stamp.sec     = static_cast<int32_t>(td.header.stamp_ns / 1'000'000'000ULL);
          ts.header.stamp.nanosec = static_cast<uint32_t>(td.header.stamp_ns % 1'000'000'000ULL);
          ts.child_frame_id       = td.child_frame_id;
          ts.transform.translation.x = td.translation.x();
          ts.transform.translation.y = td.translation.y();
          ts.transform.translation.z = td.translation.z();
          ts.transform.rotation.x = td.rotation.x();
          ts.transform.rotation.y = td.rotation.y();
          ts.transform.rotation.z = td.rotation.z();
          ts.transform.rotation.w = td.rotation.w();
          try { _tf_buffer->setTransform(ts, "loader", true); } catch (...) {}
        }
        if (writer) writer->write(msg->topic, msg->log_stamp_ns, *msg);
        continue;
      }
      if (auto msg = std::dynamic_pointer_cast<Message_<InternalCloudData>>(anon_msg)) {
        if (_lidar_frame.empty()) _lidar_frame = msg->header.frame_id;
        _tryResolveTF();
        auto f = fromInternalCloudMsg(msg); 
        ++num_clouds;
        if (f) {
          f->imus         = imus;        imus.clear();
          f->odometries   = odometries;  odometries.clear();
          f->T_base_lidar = transformIso_<Dim, Scalar>(_T_base_lidar);
          f->T_base_imu   = transformIso_<Dim, Scalar>(_T_base_imu);
          if (_frame_queue) _frame_queue->push(std::move(f));
        }
        continue;
      }
      if (auto msg = std::dynamic_pointer_cast<Message_<TreeDataType>>(anon_msg)) {
        if (_lidar_frame.empty()) _lidar_frame = msg->header.frame_id;
        _tryResolveTF();
        auto f = fromTreeMsg(msg);
        ++num_trees;
        if (f) {
          f->imus         = imus;        imus.clear();
          f->odometries   = odometries;  odometries.clear();
          f->T_base_lidar = transformIso_<Dim, Scalar>(_T_base_lidar);
          f->T_base_imu   = transformIso_<Dim, Scalar>(_T_base_imu);
          if (_frame_queue) _frame_queue->push(f);
        }
        continue;
      }
      if (auto msg = std::dynamic_pointer_cast<Message_<ImuData>>(anon_msg)) {
        ++num_imus;
        if (_imu_frame.empty()) _imu_frame = msg->header.frame_id;
        if (_base_frame.empty()) _base_frame = msg->header.frame_id;
        double ts = msg->header.stamp_ns * 1e-9;
        imus[ts] = *msg;
        if (writer) writer->write(msg->topic, msg->header.stamp_ns, *msg);
        continue;
      }
      if (auto msg = std::dynamic_pointer_cast<Message_<OdometryData>>(anon_msg)) {
        ++num_odoms;
        if (_base_frame.empty()) _base_frame = msg->child_frame_id;
        double ts = msg->header.stamp_ns * 1e-9;
        odometries[ts] = *msg;
        if (writer) writer->write(msg->topic, msg->header.stamp_ns, *msg);
        continue;
      }
    }
    if (writer) writer->close();
    if (_frame_queue) _frame_queue->setDone();
    reader_thread.join();
    voxelizer_thread.join();
    _running = false;
  }

} // namespace kd_slam
