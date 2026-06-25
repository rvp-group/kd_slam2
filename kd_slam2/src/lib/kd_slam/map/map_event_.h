#pragma once
#include "map_common.h"
#include "kd_slam/tree/tree_.h"
#include "kd_slam/event/event.h"
#include "kd_slam/icp/icp_stats_.h" 
#include <map>
#include <vector>
#include <iomanip>
namespace kd_slam {
  namespace map {
    using namespace event;
    
    template <typename NodeType_>
    struct Event_: public event::EventBase {
      using NodeType = NodeType_;
      using Scalar       = typename NodeType_::Scalar;
      using VectorType   = typename NodeType_::VectorType;
      using IsometryType = typename NodeType_::IsometryType;
      using ICPStats     = kd_slam::icp::ICPStats_<Scalar>;

      double ts = 0;
      Event_(double ts_=0): ts(ts_) {}
    };

    enum FrameAddedReason {FrameAddedPlaceholder, FrameAddedCloud, FrameAddedDeskew, FrameAddedReplay};
    
    template <typename NodeType_>
    struct EventFrameAdded_: public Event_<NodeType_> {
      using NodeType     = NodeType_;
      using VectorType   = typename NodeType_::VectorType;
      using IsometryType = typename NodeType_::IsometryType;
      using TreeBaseType = Tree_<NodeType>;
      using TreeBasePtr  = std::shared_ptr<TreeBaseType>;
      EventFrameAdded_(double ts_,
                       int count_,
                       int kf_ref_,
                       const typename NodeType::IsometryType& pose_in_kf_,
                       const std::vector<typename NodeType::VectorType>& compressed_leaves_,
                       FrameAddedReason reason_,
                       const std::string& topic_ = {},
                       uint64_t bag_offset_ = 0,
                       uint64_t stamp_ns_ = 0):
        Event_<NodeType_>(ts_),
        kf_ref(kf_ref_),
        count(count_),
        pose_in_kf(pose_in_kf_),
        compressed_leaves(compressed_leaves_),
        reason(reason_),
        topic(topic_),
        bag_offset(bag_offset_),
        stamp_ns(stamp_ns_){}
      int kf_ref;
      int count;
      IsometryType pose_in_kf;
      std::vector<VectorType> compressed_leaves;
      FrameAddedReason reason;
      std::string topic;
      uint64_t bag_offset = 0;
      uint64_t stamp_ns = 0;
      void print(std::ostream& os) const override {
        using namespace std;
        os << "\r[FRAME]: #" << fixed << setprecision(3) << this->ts
           << " #" << count
           << " kf_ref: " << kf_ref;
      }
      
      virtual TreeBasePtr getTree() { return _tree; }
      virtual void setTree(TreeBasePtr t) {_tree=t;}
      TreeBasePtr _tree;
    };

    template <typename NodeType_>
    struct EventKeyframeAdded_: public Event_<NodeType_> {
      using NodeType     = NodeType_;
      using Scalar       = typename NodeType::Scalar;
      using IsometryType = typename NodeType::IsometryType;
      using ICPStats     = typename Event_<NodeType_>::ICPStats;
      EventKeyframeAdded_(double ts_, int
                          kf_ref_,
                          int prev_kf_,
                          const IsometryType& pose_global_,
                          const ICPStats& stats_):
        Event_<NodeType_>(ts_),
        kf_ref(kf_ref_),
        prev_kf(prev_kf_),
        pose_global(pose_global_),
        stats(stats_){}
      int kf_ref;
      int prev_kf;
      IsometryType pose_global;
      ICPStats stats;
      void print(std::ostream& os) const override {
        using namespace std;
        int num_leaves=stats.num_inliers+stats.num_outliers+stats.num_bad;
        Scalar inlier_frac=(Scalar)stats.num_inliers/(Scalar)num_leaves;
        os << "\r[KEYFRAME]:" << fixed << setprecision(3) << this->ts
           << " old_ref: " << prev_kf
           << " new_ref: " << kf_ref
           << " PiW: " << pose_global.translation().transpose()
           << " inl: " << stats.num_inliers << "/" << num_leaves
           << " (" << fixed << setprecision(1) << inlier_frac * 100.f << "%)"
           << " chik: " << stats.chi2_kernelized
           << endl;
      }
    };

    template <typename NodeType_>
    struct EventOdometry_: public Event_<NodeType_> {
      using NodeType     = NodeType_;
      using Scalar       = typename NodeType::Scalar;
      using IsometryType = typename NodeType::IsometryType;
      using ICPStats     = typename Event_<NodeType_>::ICPStats;
      using VelocityVectorType = typename NodeType::Traits::GeometryTraits::VelocityVectorType;
      EventOdometry_(double ts_,
                     const IsometryType& pose_in_kf_,
                     const IsometryType& pose_global_,
                     const ICPStats& stats_):
        Event_<NodeType_>(ts_),
        pose_in_kf(pose_in_kf_),
        pose_global(pose_global_),
        stats(stats_){}
      IsometryType pose_in_kf;
      IsometryType pose_global;
      ICPStats stats;
      void print(std::ostream& os) const override {
        using namespace std;
        os << " | ODOM:" << fixed << setprecision(3)
           << " PiK: " << pose_in_kf.translation().transpose()
           << " PiW: " << pose_global.translation().transpose()
           << " inl: " << stats.inlierRatio()
           << " (" << fixed << setprecision(1) << stats.inlierRatio() * 100.f << "%)"
           << " chik: " << stats.chi2_kernelized;

      }
    };

    template <typename NodeType_>
    struct EventVelocity_: public Event_<NodeType_> {
      using NodeType     = NodeType_;
      using Scalar       = typename NodeType::Scalar;
      using IsometryType = typename NodeType::IsometryType;
      using ICPStats     = typename Event_<NodeType_>::ICPStats;
      using VelocityVectorType = typename NodeType::Traits::GeometryTraits::VelocityVectorType;
      EventVelocity_(double ts_,
                     int count_,
                     VelocityVectorType vel):
        Event_<NodeType_>(ts_),
        count(count_),
        velocity(vel){}
      int count=-1;
      VelocityVectorType velocity = VelocityVectorType::Zero();
      void print(std::ostream& os) const override {
        using namespace std;
        os << "VEL" << fixed << setprecision(3) << velocity.transpose() <<endl;
      }
    };

    template <typename NodeType_>
    struct EventFrameProcessed_: public Event_<NodeType_> {
      using NodeType     = NodeType_;
      KDStatus   status;
      double       t_align = 0;
      double       t_prop  = 0;
      double       t_loop  = 0;
      double       t_rel   = 0;
      double       t_opt   = 0;
      double       pending_chi = 0;
      EventFrameProcessed_(KDStatus st, double ta, double tp, double tl, double tr, double to=0, double pchi=0):
        status(st),
        t_align(ta),
        t_prop(tp),
        t_loop(tl),
        t_rel(tr),
        t_opt(to),
        pending_chi(pchi){}

      void print(std::ostream& os) const override {
        using namespace std;
        os << " " << KDStatusStr[status]
           << " ta:" << setprecision(1) << t_align
           << " tp:" << t_prop
           << " tl:" << t_loop
           << " tr:" << t_rel
           << " to:" << t_opt
           << " pchi:" << pending_chi << "\n";
      }
    };

    struct EventFactorChanged: public event::EventBase {
      size_t key;
      bool enabled;
      EventFactorChanged(size_t k, bool e): key(k), enabled(e){}
      void print(std::ostream& os) const override {
        using namespace std;
        os << " FactorChanged key: " << key << " status:" << enabled << endl;
      }
    };
    
    template <typename NodeType_>
    struct EventFactorAdded_: public Event_<NodeType_> {
      using NodeType        = NodeType_;
      using ICPStats        = typename Event_<NodeType_>::ICPStats;
      using IsometryType    = typename NodeType::IsometryType;
      using PoseHessianType = typename NodeType::PoseHessianType;

      ICPStats stats;
      int from_ref=1;
      int to_ref=-1;
      IsometryType Z_from_to=IsometryType::Identity();
      PoseHessianType omega_from_to=PoseHessianType::Zero();
      KDFactorType type=Odometry;
      size_t key;
      bool is_enabled=true;
      EventFactorAdded_(double ts_,
                        int f_ref,
                        int t_ref,
                        const IsometryType& Z_f_t,
                        const PoseHessianType& om_f_t,
                        const ICPStats& sta,
                        KDFactorType ty,
                        size_t k): 
        Event_<NodeType_>(ts_)
      {
        from_ref=f_ref;
        to_ref=t_ref;
        Z_from_to=Z_f_t;
        omega_from_to=om_f_t;
        stats=sta;
        type =ty;
        key = k;
      }

      void print(std::ostream& os) const override {
        const char* types_str[]={"Odometry", "Relocalize", "Loop"};
        os << "\r[FACTOR" << types_str[type] << "] " << from_ref << " <-> " << to_ref << "\n";
      }
    };

    
    template <typename NodeType_>
    struct EventPGOOptimized_: public Event_<NodeType_> {
      using NodeType     = NodeType_;
      using IsometryType = typename NodeType::IsometryType;
      std::map<int, IsometryType> poses_after;
      std::string message;
      EventPGOOptimized_(double ts_, std::map<int, IsometryType> poses, const std::string& msg=""):
        Event_<NodeType_>(ts_),
        poses_after(std::move(poses)),
        message(msg)
      {}
      void print(std::ostream& os) const override {
        os << "\r[PGO] optimized " << poses_after.size() << " frames" << "\n";
      }
    };

    template <typename NodeType_>
    struct EventLoopSearch_: public Event_<NodeType_> {
      
      using NodeType = NodeType_;
      using Scalar   = typename NodeType::Scalar;
      EventLoopSearch_(double ts_) :
        Event_<NodeType_>(ts_)
      {}
      
      void print(std::ostream& os) const override {
        os << " | lp_sp: " << matches.size();
      }
      std::vector<LoopMatch> matches;
    };

    template <typename NodeType_>
    struct EventRelocalize_: public Event_<NodeType_> {
      using NodeType = NodeType_;
      std::vector<int> local_map_refs;
      int              matched_ref = -1;
      EventRelocalize_(double ts_):
        Event_<NodeType_>(ts_)
      {}
      void print(std::ostream& os) const override {
        os << "\r[RELOC] matched kf " << matched_ref
           << " | spatial candidates: " << local_map_refs.size() <<"\n";
      }
    };

  }
}
