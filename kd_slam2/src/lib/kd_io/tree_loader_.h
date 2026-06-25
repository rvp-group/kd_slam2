#pragma once
#include "kd_io/message_reader.h"
#include "kd_io/message_writer.h"
#include "kd_io/slam_messages.h"
#include "kd_slam/tree/tree_generator_.h"
#include "kd_slam/utils/voxelizer_.h"
#include "kd_slam/frame/frame_queue_base.h"
#include "kd_slam/frame/frame_tree_.h"
#include "kd_slam/utils/runnable.h"
#include "srrg_config/configurable.h"
#include "srrg_config/property_configurable.h"
#include <memory>
#include <thread>
#include <functional>

namespace tf2 { class BufferCore; }

namespace kd_slam {

  template <typename NodeType_>
  struct TreeLoader_ : public kd_slam::utils::Runnable, public srrg2_core::Configurable {
    using LoaderTraits        = KDTreeLoaderTraits_<NodeType_>;
    using TreeType            = typename LoaderTraits::TreeType;
    using PointCloudDataType  = typename LoaderTraits::PointCloudDataType;
    using TreeDataType        = typename LoaderTraits::TreeDataType;
    using ThisType            = TreeLoader_<NodeType_>;
    using TreeCPUType         = TreeCPU_<TreeType>;
    using NodeType            = typename TreeType::NodeType;
    using Scalar              = typename TreeType::Scalar;
    using PointTraits         = typename NodeType::Traits;
    using VoxelizerType       = utils::Voxelizer_<PointTraits>;
    using GeneratorType       = TreeGenerator_<NodeType>;
    using FrameTree           = frame::FrameTree_<NodeType>;
    using PointCloudType      = std::vector<typename PointTraits::PointType>;
    static constexpr int Dim  = NodeType_::Dim;
    
    PARAM(srrg2_core::PropertyBool,                        verbose,   "print progress to stderr",             false,      nullptr);
    PARAM(srrg2_core::PropertyString,                      out_topic, "topic for output trees",               "/kdtrees", nullptr);
    PARAM(srrg2_core::PropertyConfigurable_<MessageReader>, reader,   "message reader",                       nullptr,    nullptr);
    PARAM(srrg2_core::PropertyConfigurable_<MessageWriter>, writer,   "output writer (empty = disabled)",     nullptr,    nullptr);
    PARAM(srrg2_core::PropertyConfigurable_<VoxelizerType>, voxelizer, "voxelizer",                          nullptr,    nullptr);
    PARAM(srrg2_core::PropertyConfigurable_<GeneratorType>, generator, "tree generator",                     nullptr,    nullptr);

    ~TreeLoader_();

    void setFrameQueue(frame::FrameQueueBase& fq) { _frame_queue = &fq; }

    std::shared_ptr<FrameTree> getFrame(const std::string& topic,
                                        uint64_t offset,
                                        uint64_t stamp_ns);

    void run(bool rn = true) override;
    void join() { if (_runner_thread.joinable()) _runner_thread.join(); }

    std::function<void(PointCloudType&, double)> on_cloud;

  protected:
    std::thread             _runner_thread;
    frame::FrameQueueBase*  _frame_queue = nullptr;

    std::unique_ptr<tf2::BufferCore> _tf_buffer;
    Eigen::Isometry3d  _T_base_lidar    = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d  _T_base_imu      = Eigen::Isometry3d::Identity();
    bool               _tf_resolved     = false;
    std::string        _lidar_frame;
    std::string        _imu_frame;
    std::string        _base_frame;

    void _tryResolveTF();

    std::shared_ptr<FrameTree> fromCloudMsg(std::shared_ptr<Message_<PointCloudDataType>> msg);
    std::shared_ptr<FrameTree> fromTreeMsg(std::shared_ptr<Message_<TreeDataType>> msg);
    void _runner();
  };

} // namespace kd_slam
