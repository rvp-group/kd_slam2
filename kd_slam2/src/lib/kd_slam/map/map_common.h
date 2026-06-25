#pragma once
#include <Eigen/Core>
#include <cmath>

namespace kd_slam {
  struct MatchThresholds {
    using Scalar = float;
    Scalar max_chi2_spatial = 30.f;
    Scalar min_inlier_ratio = 0.8f;
    Scalar min_score        = 0.1f;
    int    min_hops         = -1;   // -1 = no lower bound
    int    max_hops         = -1;   // -1 = no upper bound
    Scalar slack            = 1.5f;
  };

  struct TrackerParams {
    using Scalar = float;
    Scalar min_inlier_frac       = 0.6f;
    Scalar max_kf_trans          = 3.0f;
    Scalar max_kf_rot_rad        = 2*M_PI;
  };

  struct OptimizerParams {
    using Scalar = float;
    Scalar pgo_chi2_threshold     = 0.1f;
    Scalar velocity_prior_info    = 1e3f;
    Scalar imu_gravity_prior_info = 1e3f;
  };

  struct BAParams {
    using Scalar = float;
    Scalar ba_range                = 10.f;
    Scalar ba_disable_inlier_ratio = 0.3f;
  };

  struct SLAMParams {
    using Scalar = float;
    MatchThresholds relocalize_thresholds       = {100.f, 0.7f, 0.1f, -1,  5,  1.5f};
    MatchThresholds loop_thresholds             = {100.f, 0.7f, 0.1f, 20, -1,  1.5f};
    Scalar loop_max_orientation_rad             = (M_PI / 180.f) * 10.f;
    Scalar loop_consensus_max_orientation_rad   = (M_PI / 180.f) * 10.f;
    Scalar loop_consensus_max_translation       = 0.5;
    //Scalar loop_max_translation               = 10.f;
    Scalar covariance_propagate_trans_cond_diag = 0.05f;
    Scalar covariance_propagate_rot_cond_diag   = 0.001f;
  };

  struct LocalizerParams {
    using Scalar = float;
    Scalar ba_range = 10.f;
    MatchThresholds relocalize_thresholds = {100.f, 0.7f, 0.1f, -1, 5, 1.5f};
  };
    
  enum KDFactorType {
    Odometry,
    Relocalize,
    Loop,
    ICPMultiView,
    CTICPMultiView,
    IFTracking
  };

  static constexpr const char* KDFactorTypeStr[] = {
    "Odometry",
    "Relocalize",
    "Loop",
    "ICPMultiView",
    "CTICPMultiView",
    "IFracking"
  };
    
  enum KDStatus {
    Init,
    Tracking,
    Relocalized,
    Lost,
    ICPBundling,
    CTICPBundling,
    Ready
  };

  static constexpr const char* KDStatusStr[] = {
    "Init",
    "Tracking",
    "Relocalizing",
    "Lost",
    "ICPBundling",
    "CTICPBundling",
    "Ready"
  };
    
  enum MatchLabelResult {
    MatchOk,
    MatchFailChi2Spatial,
    MatchFailHops,
    MatchFailInlierRatio,
    MatchFailScore,
    MatchFailFinite,
    MatchDescriptorFail,
    MatchLoopConsensusFail,
    MatchLoopGraphFail,
    MatchUndetermined
  };

  static constexpr const char* MatchLabelResultStr[] = {
    "MatchOk",
    "MatchFailChi2Spatial",
    "MatchFailHops",
    "MatchFailInlierRatio",
    "MatchFailScore",
    "MatchFailFinite",
    "MatchDescriptorFail",
    "MatchLoopConsensusFail",
    "MatchLoopGraphFail",
    "MatchUndetermined"
  };

  struct LoopMatch {
    using Scalar=float;
    int ref=-1;
    Scalar chi2_spatial=-1;
    Scalar descriptor_distance=-1;
    Scalar rotation_delta=-1;
    Scalar translation_delta=-1;
    Scalar score=-1;
    Scalar inlier_ratio=-1;
    MatchLabelResult match_result=MatchUndetermined;
  };

}
