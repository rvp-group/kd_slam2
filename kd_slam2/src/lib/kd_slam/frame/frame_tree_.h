#pragma once
#include "kd_slam/frame/frame_base.h"
#include "kd_slam/tree/tree_.h"
#include "kd_slam/tree/tree_convert_.h"
#include "kd_slam/tree/tree_cpu_.h"
#include "kd_io/slam_message_data.h"

namespace kd_slam {
  namespace frame {
    template <typename Node_>
    struct FrameTree_: public FrameBase{
      using NodeType         = Node_;
      using TreeBaseType     = Tree_<NodeType>;
      using FrameTree        = FrameTree_<NodeType>;
      using Ptr              = std::shared_ptr<FrameTree>;
      using VelocityVectorType = typename NodeType::GeometryTraits::VelocityVectorType;
      using IsometryType = typename Node_::IsometryType;
      std::shared_ptr<TreeBaseType> tree;
      std::string topic;
      uint64_t bag_offset = 0;
      uint64_t stamp_ns = 0;
      std::map<double, ImuData> imus;
      std::map<double, OdometryData> odometries;
      IsometryType T_base_lidar = IsometryType::Identity();
      IsometryType T_base_imu   = IsometryType::Identity();
      using FrameBase::ts;

      FrameTree_() {ts=0;}
      FrameTree_(double ts_, std::shared_ptr<TreeBaseType> tree_):
        tree(tree_){
        ts=ts_;
      }
      
      bool isGPU() const {
        return tree? tree->isGPU() : false;
      }
      
      static Ptr toCompute(Ptr src, bool to_gpu) {
        if (! src)
          return nullptr;
        auto ret = std::make_shared<FrameTree>(*src);
        ret->tree=Tree_toCompute(src->tree, to_gpu);
        return ret;
      }
    };
    
    template <typename Node_>
    using FrameTreePtr_=std::shared_ptr<FrameTree_<Node_> >;
  }
}
