#pragma once
#include "kd_slam/frame/frame_tree_.h"
#include "srrg_boss/serializable.h"

struct RosHeader;

// ---------------------------------------------------------------------------
// Base + wrapper
// ---------------------------------------------------------------------------
struct MessageBase: public srrg2_core::Serializable {
  uint64_t    file_offset   = 0;
  uint64_t    log_stamp_ns  = 0;
  std::string topic;
  virtual ~MessageBase() = default;
  virtual RosHeader& getHeader() = 0;
  void serialize(srrg2_core::ObjectData& data, srrg2_core::IdContext& context) override;
  void deserialize(srrg2_core::ObjectData& data, srrg2_core::IdContext& context) override;

};

template<typename T>
struct Message_ : public MessageBase, public T {
  using DataType = T;
  RosHeader& getHeader() override {return T::header;}
  Message_()=default;
  Message_(const T& base): T(base){}
  void serialize(srrg2_core::ObjectData& odata, srrg2_core::IdContext& context) override {
    MessageBase::serialize(odata, context);
    DataType::serialize(odata, context);
  }
  void deserialize(srrg2_core::ObjectData& odata, srrg2_core::IdContext& context) override{
    MessageBase::deserialize(odata, context);
    DataType::deserialize(odata, context);
  }
  
};


template <typename NodeType>
struct KDTreeLoaderTraits_{
};

template <>
struct KDTreeLoaderTraits_<kd_slam::NodePoint2f> {
  using NodeType = kd_slam::NodePoint2f;
  using TreeBaseType = kd_slam::Tree_<NodeType>;
  using TreeType = kd_slam::TreeCPU_<TreeBaseType>;
  using FrameTree      = kd_slam::frame::FrameTree_<NodeType>;
  using TreeDataType   = KDTreeData2f;
  using PointCloudDataType = PointCloudXYTData;
};


template <>
struct KDTreeLoaderTraits_<kd_slam::NodePoint3f> {
  using NodeType       = kd_slam::NodePoint3f;
  using TreeBaseType   = kd_slam::Tree_<NodeType>;
  using TreeType       = kd_slam::TreeCPU_<TreeBaseType>;
  using FrameTree      = kd_slam::frame::FrameTree_<NodeType>;
  using TreeDataType   = KDTreeData3f;
  using PointCloudDataType = PointCloudXYZTData;
};

// ---------------------------------------------------------------------------
// toMessage: wrap a TreeCPU into a KDTreeDataXf message (no points copied)
// ---------------------------------------------------------------------------
template <typename KDTreeMsgType, typename KDTreeType>
static inline KDTreeMsgType makeKDTreeMessage(const KDTreeType&  src,
                                               const RosHeader&   header,
                                               const std::string& src_topic) {
  KDTreeMsgType msg;
  msg.header    = header;
  msg.src_topic = src_topic;
  msg.tree.root_mean         = src.root_mean;
  msg.tree.root_eigenvalues  = src.root_eigenvalues;
  msg.tree.root_eigenvectors = src.root_eigenvectors;
  msg.tree._ts_start         = src._ts_start;
  msg.tree._nodes_storage    = src._nodes_storage;
  msg.tree._nodes_ptr        = msg.tree._nodes_storage.empty()
                                   ? nullptr : msg.tree._nodes_storage.data();
  msg.tree._num_nodes        = src._num_nodes;
  msg.tree._points_storage   = nullptr;
  msg.tree._points_ptr       = nullptr;
  msg.tree._num_points       = 0;
  return msg;
}

inline KDTreeData2f toMessage(const kd_slam::TreeCPUPoint2f& tree,
                               const RosHeader&               header,
                               const std::string&             src_topic) {
  return makeKDTreeMessage<KDTreeData2f>(tree, header, src_topic);
}

inline KDTreeData3f toMessage(const kd_slam::TreeCPUPoint3f& tree,
                               const RosHeader&               header,
                               const std::string&             src_topic) {
  return makeKDTreeMessage<KDTreeData3f>(tree, header, src_topic);
}

