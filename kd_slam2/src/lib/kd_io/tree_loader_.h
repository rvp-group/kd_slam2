#pragma once
#include "kd_io/message_reader.h"
#include "kd_io/message_writer.h"
#include "kd_io/slam_messages.h"
#include "kd_io/message_queue.h"
#include "kd_slam/tree/tree_generator_.h"
#include "kd_slam/utils/voxelizer_.h"
#include "kd_slam/frame/frame_queue_base.h"
#include "kd_slam/frame/frame_tree_.h"
#include "kd_slam/utils/runnable.h"
#include "kd_slam/event/event.h"
#include "kd_slam/map/map_event_.h"
#include <kd_io/static_tf_buffer.h>
#include "srrg_config/configurable.h"
#include "srrg_config/property_configurable.h"
#include <memory>
#include <thread>
#include <functional>

namespace kd_slam {

  template <typename NodeType_>
  struct TreeLoader_ :
    public kd_slam::event::EventPublisher,
    public kd_slam::utils::Runnable,
    public srrg2_core::Configurable {
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

    using EventLoadTree     = typename kd_slam::map::EventLoadTree_<NodeType_>;
    
    struct InternalCloudData  {
      RosHeader header;
      std::vector<typename NodeType::Traits::PointType> points;
      void serialize(srrg2_core::ObjectData& data, srrg2_core::IdContext& context){}
      void deserialize(srrg2_core::ObjectData& data, srrg2_core::IdContext& context){}
   };

    PARAM(srrg2_core::PropertyBool,                        verbose,   "print progress to stderr",             false,      nullptr);
    PARAM(srrg2_core::PropertyString,                      out_topic, "topic for output trees",               "/kdtrees", nullptr);
    PARAM(srrg2_core::PropertyConfigurable_<MessageReader>, reader,   "message reader",                       nullptr,    nullptr);
    PARAM(srrg2_core::PropertyConfigurable_<MessageWriter>, writer,   "output writer (empty = disabled)",     nullptr,    nullptr);
    PARAM(srrg2_core::PropertyConfigurable_<VoxelizerType>, voxelizer, "voxelizer",                          nullptr,    nullptr);
    PARAM(srrg2_core::PropertyConfigurable_<GeneratorType>, generator, "tree generator",                     nullptr,    nullptr);
    PARAM(srrg2_core::PropertyFloat,                        vertical_angle_offset_deg,   "pitch offset of the lidar (from KISS-ICP)",             0,      nullptr);

    ~TreeLoader_();

    void setFrameQueue(frame::FrameQueueBase& fq) { _frame_queue = &fq; }

    std::shared_ptr<FrameTree> getFrame(const std::string& topic,
                                        uint64_t offset,
                                        uint64_t stamp_ns);

    void run(bool rn = true) override;
    void join() { if (_runner_thread.joinable()) _runner_thread.join(); }

    std::function<void(PointCloudType&, double)> on_cloud;

  protected:
    kd_io::StaticTFBuffer      _tf_buffer;
    std::thread             _runner_thread;
    frame::FrameQueueBase*  _frame_queue = nullptr;

    Eigen::Isometry3f  _T_base_lidar    = Eigen::Isometry3f::Identity();
    Eigen::Isometry3f  _T_base_imu      = Eigen::Isometry3f::Identity();
    bool               _tf_resolved     = false;
    std::string        _lidar_frame;
    std::string        _imu_frame;
    std::string        _base_frame;
    bool               _tf_lidar_warning=false;   
    bool               _tf_imu_warning=false;   
    void _tryResolveTF();

    void applyVerticalAngleCorrection(std::vector<typename PointTraits::PointType>& pts);
    
    std::shared_ptr<FrameTree> fromInternalCloudMsg(std::shared_ptr<Message_<InternalCloudData>> msg);
    
    std::shared_ptr<FrameTree> fromCloudMsg(std::shared_ptr<Message_<PointCloudDataType>> msg);
    std::shared_ptr<FrameTree> fromTreeMsg(std::shared_ptr<Message_<TreeDataType>> msg);
    void _reader_runner();
    void _voxelizer_runner();
    void _runner();
    MessageQueueBounded _reader_queue;
    MessageQueueBounded _voxelizer_queue;
    double t_read = 0; 
    double t_tree = 0;
    double t_vox = 0;
    double t_read_cum = 0; 
    double t_tree_cum = 0;
    double t_vox_cum = 0;
    double t_read_msg = 0;
 };

} // namespace kd_slam
