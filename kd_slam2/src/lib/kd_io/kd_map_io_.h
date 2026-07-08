#pragma once
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <list>
#include "kd_slam/map/map_.h"
#include "kd_slam/map/multiview_icp_factor_.h"
#include "kd_slam/map/multiview_cticp_factor_.h"
#include "kd_slam/map/map_common.h"
#include "tree_loader_.h"
#include "reader_factory.h"

namespace kd_slam {
  using namespace event;
  using namespace map;
  
  template <typename NodeType_>
  std::shared_ptr<Map_<NodeType_>> loadMap(const std::string& filename_prefix, bool is_gpu) {
    static constexpr int  MAX_FRAMES=Map_<NodeType_>::MAX_FRAMES;
    
    using MapType = Map_<NodeType_>;
    using FrameTree       = typename MapType::FrameTree;
    using Frame           = typename MapType::Frame;
    using DescriptorType             = typename MapType::DescriptorType;
    using PGOFactor                  = typename MapType::PGOFactor;
    using MultiViewICPFactor         = typename MapType::MultiViewICPFactor;
    using MultiViewCTICPFactor       = typename MapType::MultiViewCTICPFactor;

    using SolverVelocityVariableType = typename MapType::SolverVelocityVariableType;
    using SolverVelocityPriorFactorType  = typename MapType::SolverVelocityPriorFactorType;
    using SolverGravityPriorFactorType   = typename MapType::SolverGravityPriorFactorType;
    using SolverVariableType         = typename MapType::SolverVariableType;
    using SolverPGOFactorType        = typename MapType::SolverPGOFactorType;
    using SolverMultiviewICPFactorType   = map::MultiViewICPFactor_<NodeType_>;
    using SolverMultiviewCTICPFactorType = map::MultiViewCTICPFactor_<NodeType_>;

    using GeometryTraits             = typename NodeType_::Traits::GeometryTraits;

    
    std::string graph_filename = filename_prefix + std::string("_map.boss");
    std::string tree_filename  = filename_prefix + std::string("_map_kd");
    auto frame_graph=std::make_shared<MapType>();
    auto factor_graph=FactorGraph::read(graph_filename);
    frame_graph->_factor_graph = factor_graph;
    
    std::map<int, SolverVelocityVariableType*> vel_vars;
    std::map<int, SolverVelocityPriorFactorType*>  vel_priors;
    std::map<int, SolverGravityPriorFactorType*>          gravity_priors;
    std::map<int, SolverVariableType*>          pose_vars;

    for (auto v_it : factor_graph->variables()) {
      if (auto v_pose = dynamic_cast<SolverVariableType*>(v_it.second)) {
        pose_vars[v_pose->graphId()] = v_pose;
      }
      if (auto v_vel = dynamic_cast<SolverVelocityVariableType*>(v_it.second)) {
        vel_vars[v_vel->graphId() - MAX_FRAMES] = v_vel;
      }
    }
    for (auto f_it : factor_graph->factors()) {
      if (auto f_prior = dynamic_cast<SolverVelocityPriorFactorType*>(f_it.second)) {
        vel_priors[f_prior->variableId(0) - MAX_FRAMES] = f_prior;
      }
      if constexpr (NodeType_::Dim == 3) {
        if (auto f_gprior = dynamic_cast<SolverGravityPriorFactorType*>(f_it.second)) {
          gravity_priors[f_gprior->variableId(0)] = f_gprior;
          /*
          f_gprior->setInformationMatrix(SolverGravityPriorFactorType::InformationMatrixType::Identity()*params.imu_gravity_prior_info);
          */
        }
      }
    }
    
    // read keyframe trees from BAG and populate _frame_graph
    using LoaderTraits = KDTreeLoaderTraits_<NodeType_>;
    using TreeDataType = typename LoaderTraits::TreeDataType;
    using TreeCPUType  = typename LoaderTraits::TreeType;
    auto reader = makeRosbag2Reader(tree_filename, {"/keyframes"});
    reader->open();
    while (reader->isGood()) {
      auto anon = reader->readOne();
      if (!anon) break;
      auto msg = std::dynamic_pointer_cast<Message_<TreeDataType>>(anon);
      if (!msg) continue;
      int ref = std::stoi(msg->src_topic);
      auto pose_it = pose_vars.find(ref);
      if (pose_it == pose_vars.end()) continue;
      auto tree_cpu = std::make_shared<TreeCPUType>(msg->tree);
      auto ft_src = std::make_shared<FrameTree>(msg->header.stamp_ns * 1e-9, tree_cpu);
      auto ft = FrameTree::toCompute(ft_src, is_gpu);
      auto frame = std::make_shared<Frame>(*ft,
                                           DescriptorType{},
                                           pose_it->second->estimate(),
                                           ref);
      frame->solver_variable=pose_it->second;
      auto vv_it=vel_vars.find(ref);
      if (vv_it!=vel_vars.end()) {
        frame->solver_velocity=vv_it->second;
      } 

      auto vp_it=vel_priors.find(ref);
      if (vp_it!=vel_priors.end()) {
        frame->solver_velocity_prior=vp_it->second;
      } 

      auto gp_it=gravity_priors.find(ref);
      if (gp_it!=gravity_priors.end()) {
        frame->solver_gravity_prior=gp_it->second;
      }
        
      frame_graph->addFrame(frame);
      assert(ref==frame->ref() && "ref mismatch");
    }
    for (auto f_it : factor_graph->factors()) {
      auto f = f_it.second;
      if (auto sf = dynamic_cast<SolverPGOFactorType*>(f)) {
        // reconstruct PGOFactor wrapper
        auto pgo = std::make_shared<PGOFactor>();
        pgo->from_ref       = sf->variableId(0);
        pgo->to_ref         = sf->variableId(1);
        pgo->Z_from_to      = sf->measurement();
        pgo->Z_to_from      = pgo->Z_from_to.inverse();
        pgo->omega_from_to  = sf->informationMatrix();
        pgo->omega_to_from  = GeometryTraits::transformOmega(pgo->omega_from_to, pgo->Z_from_to);
        pgo->det_from_to    = pgo->omega_from_to.determinant();
        pgo->det_to_from    = pgo->omega_to_from.determinant();
        pgo->type           = Odometry; // default; loop factors will also load correctly
        pgo->solver_factor  = sf;
        pgo->is_enabled       = sf->enabled();
        frame_graph->addFactor(pgo);
      } else if (auto sf = dynamic_cast<SolverMultiviewICPFactorType*>(f)) {
        // reconstruct MultiViewICPFactor wrapper with null compute ptrs
        auto kdf = std::make_shared<MultiViewICPFactor>();
        kdf->from_ref      = sf->variableId(0);
        kdf->to_ref        = sf->variableId(1);
        kdf->solver_factor = sf;
        sf->kd_factor      = kdf.get();
        kdf->is_enabled    = sf->enabled();
        kdf->is_valid      = sf->enabled();
        frame_graph->FrameGraph::addFactor(kdf);
      } else if (auto sf = dynamic_cast<SolverMultiviewCTICPFactorType*>(f)) {
        // reconstruct MultiViewCTICPFactor wrapper with null compute ptrs
        auto kdf = std::make_shared<MultiViewCTICPFactor>();
        kdf->from_ref      = sf->variableId(0);
        kdf->vel_from_ref  = sf->variableId(1);
        kdf->to_ref        = sf->variableId(2);
        kdf->vel_to_ref    = sf->variableId(3);
        kdf->solver_factor = sf;
        kdf->is_enabled      = sf->enabled();
        kdf->is_valid      = sf->enabled();
        sf->kd_factor      = kdf.get();
        frame_graph->FrameGraph::addFactor(kdf);
      }
    }
    return frame_graph;
  }

  template <typename NodeType_>
  void saveMap(const std::string& filename_prefix, const Map_<NodeType_>& m) {
    std::string graph_filename = filename_prefix + std::string("_map.boss");
    std::string tree_filename  = filename_prefix + std::string("_map_kd");
    m._factor_graph->write(graph_filename);
    using MapType      = Map_<NodeType_>;
    using Frame        = typename MapType::Frame;
    using LoaderTraits = KDTreeLoaderTraits_<NodeType_>;
    using TreeDataType = typename LoaderTraits::TreeDataType;
    using TreeCPUType  = typename LoaderTraits::TreeType;

    auto writer = makeRosbag2Writer(tree_filename);
    writer->open();
    for (auto [ref, frame_] : m.frames()) {
      auto frame = std::dynamic_pointer_cast<Frame>(frame_);
      if (!frame || !frame->tree) continue;
      auto tree_cpu = std::dynamic_pointer_cast<TreeCPUType>(frame->tree);
      if (!tree_cpu)
        tree_cpu = std::make_shared<TreeCPUType>(*frame->tree);
      RosHeader hdr;
      hdr.stamp_ns = (uint64_t)(frame->ts * 1e9);
      Message_<TreeDataType> wrapped;
      static_cast<TreeDataType&>(wrapped) = toMessage(*tree_cpu, hdr, std::to_string(ref));
      writer->write("/keyframes", hdr.stamp_ns, wrapped);
    }
    writer->close();
  }
  
} // namespace kd_slam
